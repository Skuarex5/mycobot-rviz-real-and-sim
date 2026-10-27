#pragma once
#include <QPointer>
#include <rviz_common/properties/property.hpp>
namespace mycobot_280_live_control {
// Preserve the user's planning display preference across live-mode transitions.
class PlanningMarkerVisibility {
public:
  ~PlanningMarkerVisibility() { restore(); }
  void update(rviz_common::properties::Property *request, bool live) {
    if (!live) {
      restore();
      return;
    }
    if (goal_ || !request)
      return;
    for (int i = 0; i < request->numChildren(); ++i) {
      auto child = request->childAt(i);
      if (child->getName() == "Query Goal State") {
        goal_ = child;
        original_ = child->getValue().toBool();
        child->setValue(false);
        return;
      }
    }
  }

private:
  void restore() {
    if (goal_)
      goal_->setValue(original_);
    goal_.clear();
  }
  QPointer<rviz_common::properties::Property> goal_;
  bool original_ = false;
};
} // namespace mycobot_280_live_control
