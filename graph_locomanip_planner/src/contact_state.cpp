#include <graph_locomanip_planner/contact_state.h>
namespace graph_locomanip_planner {
  namespace {
    bool sameCandidate(const ContactCandidate& a, const ContactCandidate& b) {
      return a.bodyName == b.bodyName && a.linkName == b.linkName && a.isStatic == b.isStatic;
    }
    bool samePosition(const ContactCandidate& a, const ContactCandidate& b) {
      return sameCandidate(a,b) && a.localPose.translation().isApprox(b.localPose.translation(),1e-9);
    }
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
        if(!matched[j] && ((samePosition(a.contacts[i].c1,b.contacts[j].c1) && samePosition(a.contacts[i].c2,b.contacts[j].c2)) ||
           (samePosition(a.contacts[i].c1,b.contacts[j].c2) && samePosition(a.contacts[i].c2,b.contacts[j].c1)))) {
          matched[j] = true; found = true; break;
        }
      }
      if(!found) return false;
    }
    return true;
  }
}
