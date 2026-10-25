#include <graph_locomanip_planner/contact_state.h>
namespace graph_locomanip_planner {
  namespace {
    bool sameCandidate(const ContactCandidate& a, const ContactCandidate& b) {
      return a == b;
    }
  }
  bool operator==(const ContactCandidate& a, const ContactCandidate& b) {
    return a.bodyName == b.bodyName && a.linkName == b.linkName && a.name == b.name &&
      a.isStatic == b.isStatic && a.localPose.toHomogeneousMatrix().isApprox(b.localPose.toHomogeneousMatrix(),1e-9);
  }
  bool operator==(const Contact& a, const Contact& b) {
    return (sameCandidate(a.c1,b.c1) && sameCandidate(a.c2,b.c2)) ||
      (sameCandidate(a.c1,b.c2) && sameCandidate(a.c2,b.c1));
  }
  bool operator==(const ContactState& a, const ContactState& b) {
    if(a.contacts.size() != b.contacts.size()) return false;
    std::vector<bool> matched(b.contacts.size(),false);
    for(std::size_t i = 0; i < a.contacts.size(); ++i) {
      bool found = false;
      for(std::size_t j = 0; j < b.contacts.size(); ++j) {
        if(!matched[j] && a.contacts[i] == b.contacts[j]) {
          matched[j] = true; found = true; break;
        }
      }
      if(!found) return false;
    }
    return true;
  }
}
