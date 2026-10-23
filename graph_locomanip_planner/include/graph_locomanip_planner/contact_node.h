#pragma once
#include <graph_search/node.h>
#include <graph_locomanip_planner/contact_state.h>
namespace graph_locomanip_planner {
  class ContactNode : public graph_search::Node {
  public:
    const ContactState& state() const { return this->state_; }
    ContactState& state() { return this->state_; }
  private:
    ContactState state_;
  };
}
