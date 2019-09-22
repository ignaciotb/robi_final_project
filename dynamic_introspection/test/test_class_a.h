/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _TEST_CLASS_A_
#define _TEST_CLASS_A_

#include "test_class_abstract.h"
#include <vector>
#include <string>

class TestClassA : public TestClassBase
{
public:
  TestClassA();

  virtual ~TestClassA();

  void update();

private:
  double cont_double_1_;
  double cont_double_2_;
  int cont_;
  std::vector<std::string> registered_ids_;
};

#endif
