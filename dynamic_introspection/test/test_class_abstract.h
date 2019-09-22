/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _TEST_CLASS_BASE_
#define _TEST_CLASS_BASE_

class TestClassBase{
public:

  virtual ~TestClassBase(){}

  virtual void update() = 0;

};

#endif
