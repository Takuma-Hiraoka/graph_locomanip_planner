#pragma once
#include <pinocchio/spatial/se3.hpp>
#include <Eigen/Core>
#include <string>
#include <vector>

namespace graph_locomanip_planner {
  class ContactCandidate {
  public:
    ContactCandidate() = default;
    ContactCandidate(const std::string& body, const std::string& link,
      const pinocchio::SE3& pose = pinocchio::SE3::Identity(), bool fixed = true)
      : bodyName(body), linkName(link), localPose(pose), isStatic(fixed) {}
    std::string bodyName, linkName;
    // Optional label for multiple contact sites on the same link.
    std::string name;
    std::vector<std::string> allowedDynamicLinks;
    std::vector<std::string> allowedStaticLinks;
    bool allowDynamicContact = true;
    bool allowStaticContact = true;
    pinocchio::SE3 localPose = pinocchio::SE3::Identity();
    bool isStatic = true;
    Eigen::Matrix<double,6,1> weight = Eigen::Matrix<double,6,1>::Ones();
    // Clockwise vertices in localPose XY, used for robot/static support.
    std::vector<Eigen::Vector2d> supportPolygon;
  };
  bool operator==(const ContactCandidate& a, const ContactCandidate& b);
  class Contact {
  public:
    Contact() = default;
    Contact(const ContactCandidate& a, const ContactCandidate& b) : c1(a), c2(b) {}
    ContactCandidate c1, c2;
    // Relative yaw in the canonical B contact frame (static side, otherwise c2).
    // Captured when contact is established; not part of contact-site identity.
    double yaw = 0.0;
    bool yawFixed = false;
  };
  bool operator==(const Contact& a, const Contact& b);
  class ContactState {
  public:
    // Model order for each configuration vector: robot, then object entries.
    std::vector<std::vector<Eigen::VectorXd> > transition;
    std::vector<Eigen::VectorXd> frame;
    std::vector<Contact> contacts;
  };
  bool operator==(const ContactState& a, const ContactState& b);
}
