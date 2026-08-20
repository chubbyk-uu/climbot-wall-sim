// Copyright 2026 jerry
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Test that an abandoned request cannot speak for the one that replaced it.
//
// The panel's three-second timeout re-enables the controls, which is right: a
// service that passes service_is_ready() and then dies would otherwise leave
// them disabled until RViz restarts. What it does not do is retract the
// request. The service can still answer, and its callback carries what it was
// going to say when it was asked - the configuration in force then, the
// tracking mode as it was then.
//
// The operator has moved on by that point; the timeout is what told them to.
// Applying the old answer clears the new request's pending flag and repaints
// the boxes to a state nobody chose, which reads as the panel undoing the
// operator's last action.

#include <gtest/gtest.h>

#include "climbot_rviz_plugins/coverage_panel.hpp"

using climbot_rviz_plugins::RequestGate;

TEST(RequestGate, AnAnswerToTheRequestInFlightIsApplied)
{
  RequestGate gate;
  EXPECT_FALSE(gate.waiting());
  const auto first = gate.begin();
  EXPECT_TRUE(gate.waiting());
  EXPECT_TRUE(gate.isCurrent(first));
  gate.settle();
  EXPECT_FALSE(gate.waiting());
}

TEST(RequestGate, AnAnswerThatArrivesAfterItsRequestWasGivenUpOnIsIgnored)
{
  RequestGate gate;
  const auto abandoned = gate.begin();
  gate.abandon();
  EXPECT_FALSE(gate.waiting());
  EXPECT_FALSE(gate.isCurrent(abandoned));
}

TEST(RequestGate, ALateAnswerCannotSettleTheRequestThatReplacedIt)
{
  // The sequence the panel actually produces: send, time out, send again, and
  // only then does the first service answer.
  RequestGate gate;
  const auto abandoned = gate.begin();
  gate.abandon();
  const auto current = gate.begin();

  EXPECT_FALSE(gate.isCurrent(abandoned));
  EXPECT_TRUE(gate.isCurrent(current));
  // The panel returns early on a false isCurrent, so the second request is
  // still in flight and its controls stay disabled until it answers.
  EXPECT_TRUE(gate.waiting());

  EXPECT_TRUE(gate.isCurrent(current));
  gate.settle();
  EXPECT_FALSE(gate.waiting());
}

TEST(RequestGate, SendingAgainRetiresAnAnswerStillInFlight)
{
  // Nothing in the panel sends a second request while one is waiting, but the
  // gate must not depend on that: it is the thing keeping the promise.
  RequestGate gate;
  const auto first = gate.begin();
  const auto second = gate.begin();
  EXPECT_NE(first, second);
  EXPECT_FALSE(gate.isCurrent(first));
  EXPECT_TRUE(gate.isCurrent(second));
}

TEST(RequestGate, AnAnswerArrivingWhenNothingIsWaitedForIsIgnored)
{
  RequestGate gate;
  const auto only = gate.begin();
  gate.settle();
  // Answered once already. A duplicate delivery must not re-open the request.
  EXPECT_FALSE(gate.isCurrent(only));
}
