#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
# TODO: License statement

import actionlib
import rospy
from control_msgs.msg import *
from sensor_msgs.msg import JointState

class SGError(Exception):
    pass

class NewGoal(Exception):
    def __init__(self, goal):
        self.goal = goal

class SimpleGraspingAction:
    """
    Simple grasping action that sits on top of a joint-space interpolating
    controller exposing the FollowJointTrajectoryAction interface.

    It exposes a FollowJointTrajectory action interface, and goals consist
    of single-waypoint trajectories with a position goal an optional effort
    threshold.
    Execution is considered successful when for every joint either the
    position goal or the effort threshold has been reached.

    Note that the action goal can specify goal constraints to override the
    controller's defaults. Path constraints will be ignored, as they apply
    only to trajectories with more than one waypoint.
    """
    target_ctrl_param = '~target_controller'
    update_rate = 30 # Hz

    def __init__(self, name):
        self._action_name = name

        # target controller info and client
        ctrl_name = self._target_ctrl_name()
        self._joints = self._target_ctrl_joints(ctrl_name)
        self._goal_tol = self._target_ctrl_goal_tol(ctrl_name, self._joints)
        self._ac = self._target_ctrl_client(ctrl_name)

        # action server
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            FollowJointTrajectoryAction,
            execute_cb = self._execute_cb,
            auto_start = False)
        self._as.start()

        # subscribers
        rospy.Subscriber("joint_states", JointState, self._joint_state_cb)
        self._joint_state = JointState()

    def _execute_cb(self, goal, eff_reached=None):
        from actionlib import SimpleGoalState
        from actionlib_msgs.msg import GoalStatus

        # validate goal message
        if not self._is_goal_valid(goal):
            ko_result = FollowJointTrajectoryResult()
            ko_result.error_code = FollowJointTrajectoryResult.INVALID_GOAL
            self._as.set_aborted(ko_result)
            return

        # update goal tolerance values
        goal_tol = self._goal_tol.copy()
        self._update_goal_tol(goal_tol, goal)

        # position and effort goals
        pos_goal = self._extract_pos_goal(goal)
        eff_goal = self._extract_eff_goal(goal)

        # forward trajectory to target controller
        self._ac.send_goal(goal)

        # wait for goal to be accepted
        rate = rospy.Rate(self.update_rate)
        while self._ac.simple_state == SimpleGoalState.PENDING and \
              not rospy.is_shutdown():
            rate.sleep()

        # estimate of trajectory end time
        end_time = rospy.Time.now() + goal.trajectory.points[0].time_from_start

        # monitor goal progress
        preempted = False
        if not eff_reached:
            eff_reached = {}
        pos_reached = {}
        try:
            while True:
                if rospy.is_shutdown():
                    return

                # check for preemption of the target controller
                if self._ac.get_state() == GoalStatus.PREEMPTED:
                    preempted = True
                    break
                # check for preemption requests to this action server
                if self._as.is_preempt_requested():
                    self._ac.cancel_goal()  # stop reaching position goal
                    preempted = True
                    break
                # check for position goal completion
                pos_reached = self._check_pos_goal_constraint(pos_goal, goal_tol)
                if self._ac.simple_state == SimpleGoalState.DONE:
                    break
                # check for effort goal completion
                eff_reached, eff_reached_new = self._update_eff_reached(eff_goal, eff_reached)
                if eff_reached and all(eff_reached.values()):
                    self._ac.cancel_goal()  # stop reaching position goal
                    break
                # send new trajectory goal that holds position for joints that have
                # just reached their effort goal
                if eff_reached_new:
                    new_goal = self._recompute_goal(goal, end_time, eff_reached_new)
                    raise NewGoal(new_goal) # TODO: can we implement the recursion in a cleaner way?
                rate.sleep()

            # wait for target controller result to reach us
            self._ac.wait_for_result()

            # send result
            if preempted:
                self._as.set_preempted()
            else:
                ok_result = FollowJointTrajectoryResult()
                ok_result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
                if self._ac.get_result().error_code == ok_result.error_code:
                    # all joints satisfy the position goal
                    self._as.set_succeeded(ok_result)
                else:
                    if eff_reached and any(eff_reached.values()):
                        # some joints satisfy the effort goal, some might also
                        # satisfy the position goal
                        if self._is_goal_reached(pos_reached, eff_reached):
                            self._as.set_succeeded(ok_result)
                        else:
                            ko_result = FollowJointTrajectoryResult()
                            ko_result.error_code = \
                                FollowJointTrajectoryResult.GOAL_TOLERANCE_VIOLATED
                            self._as.set_aborted(ko_result)
                    else:
                        # no joints satisfy the position or effort goals
                        self._as.set_aborted(self._ac.get_result())

            # log useful debug information
            log_str = "State of joint goals:"
            for joint in self._joints:
                log_str += "\n- " + joint + ": "
                if eff_reached and eff_reached[joint]:
                    log_str += "effort goal reached"
                elif pos_reached[joint]:
                    log_str += "position goal reached"
                else:
                    log_str += "no goal reached"
            rospy.logdebug(log_str)

        except NewGoal as e:
            self._execute_cb(e.goal, eff_reached) # TODO: Can we refactor so eff_reached is not a callback parameter?

    def _joint_state_cb(self, msg):
        self._joint_state = msg

    def _extract_pos_goal(self, goal):
        point = goal.trajectory.points[-1] # NOTE: Last waypoint
        return dict(zip(goal.trajectory.joint_names, point.positions))

    def _extract_eff_goal(self, goal):
        eff_goal = {}
        point = goal.trajectory.points[0] # NOTE: First waypoint
        if point.effort:
            eff_goal = dict(zip(goal.trajectory.joint_names, point.effort))
        return eff_goal

    def _update_goal_tol(self, goal_tol, msg):
        for msg_tol in msg.goal_tolerance:
            goal_tol[msg_tol.name] = msg_tol.position

    def _is_goal_valid(self, goal):
        num_points = len(goal.trajectory.points)

        if num_points != 1:
            rospy.logerr('Trajectory can only have one waypoint, goal has ' +
-                        str(num_points) + '.')
            return False
        point = goal.trajectory.points[0]

        if point.velocities or point.accelerations:
            rospy.logerr('Trajectory waypoint can only specify position and ' +
            '(optionally) effort fields, not velocities and accelerations.')
            return False
        return True

    def _check_eff_goal_constraint(self, eff_goal):
        res = {}
        for joint, eff_threshold in eff_goal.iteritems():
            idx = self._joint_state.name.index(joint)
            eff = self._joint_state.effort[idx]
            if eff_threshold >= 0.0:
                res[joint] = eff > eff_threshold
            else:
                res[joint] = eff < eff_threshold
        return res

    def _update_eff_reached(self, eff_goal, eff_reached): # TODO: Rename variables!
        eff_reached_now = self._check_eff_goal_constraint(eff_goal)
        eff_reached_new = []
        for name, val in eff_reached_now.iteritems():
            # update list of joints that reached the effort goal in this cycle
            if val and (not eff_reached.has_key(name) or not eff_reached[name]):
                eff_reached_new.append(name)
            # update global effort goal status
            if val or not eff_reached.has_key(name):
                eff_reached[name] = val
        return eff_reached, eff_reached_new

    def _recompute_goal(self, goal, end_time, hold_joints):
        goal = goal
        for name in hold_joints:
            goal_idx = goal.trajectory.joint_names.index(name)
            js_idx = self._joint_state.name.index(name)
            # NOTE: var-length arrays are deserialized as (immutable) tuples
            # need to converto to and from list to write a value :(
            pos_list = list(goal.trajectory.points[0].positions)
            pos_list[goal_idx] = self._joint_state.position[js_idx]
            goal.trajectory.points[0].positions = tuple(pos_list)
        new_tfs = end_time - rospy.Time.now()
        if new_tfs < rospy.Duration(0):
            # within the goal time tolerance region
            dt = rospy.Duration(0.01)
            goal.goal_time_tolerance -= (new_tfs + dt) # shrink goal time tolerance
            new_tfs = dt # small value in the future to ensure execution
        goal.trajectory.points[0].time_from_start = new_tfs
        return goal

    def _check_pos_goal_constraint(self, pos_goal, goal_tol):
        # NOTE: We don't use the controller's state topic, which publishes
        # tracking errors because on target controller goal preeemption
        # there might be a race in which we get the _new_ goal's error
        # and if it's a hold position trajectory, it will be very small
        res = {}
        for joint, pos_goal in pos_goal.iteritems():
            if goal_tol.has_key(joint):
                idx = self._joint_state.name.index(joint)
                pos = self._joint_state.position[idx]
                res[joint] = abs(pos - pos_goal) < goal_tol[joint]
            else:
                # no tolerance data specified, any error is acceptable
                res[joint] = True
        return res

    def _is_goal_reached(self, pos_reached, eff_reached):
        success = True
        for joint in self._joints:
            if not pos_reached[joint] or not eff_reached[joint]:
                success = False
                break
        return success

    def _target_ctrl_name(self):
        try:
            ctrl_name = rospy.get_param(self.target_ctrl_param)
        except KeyError:
            raise SGError("Parameter specifying target controller not " +
                          "found. " +
                          "Please set the '" + self.target_ctrl_param +
                          "' parameter.")
        return ctrl_name

    def _target_ctrl_joints(self, ctrl_name):
        try:
            joints_param = ctrl_name + '/joints'
            joints = rospy.get_param(joints_param)
        except KeyError:
            raise SGError("Couldn't find joints of target controller:'" +
                          + joints_param + "' parameter.")
        return joints

    def _target_ctrl_goal_tol(self, ctrl_name, joints):
        goal_tol = {}
        try:
            cons_param = ctrl_name + '/constraints'
            cons = rospy.get_param(cons_param)
        except KeyError:
            rospy.logwarn("Target controller doesn't specify goal " +
                          "constraints for any joint.")
        for joint in joints:
            try:
                goal_tol[joint] = cons[joint]['goal']
            except KeyError:
                 rospy.logwarn("Target controller doesn't specify goal " +
                               "constraints for joint '" + joint + "'.")
        return goal_tol

    def _target_ctrl_client(self, ctrl_name):
        from control_msgs.msg import FollowJointTrajectoryAction as FJTA
        ac_ns = ctrl_name + '/follow_joint_trajectory'
        ac = actionlib.SimpleActionClient(ac_ns, FJTA)
        return ac
