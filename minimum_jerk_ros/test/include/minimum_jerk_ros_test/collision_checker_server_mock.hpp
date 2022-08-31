#ifndef COLLISION_CHECKER_SERVER_MOCK_HPP
#define COLLISION_CHECKER_SERVER_MOCK_HPP

#include "cmr_tests_utils/basic_service_server_test.hpp"
#include "cmr_msgs/srv/is_collision_free.hpp"

namespace minimum_jerk_test {

class CollisionCheckerServerMock: public cmr_tests_utils::BasicServiceServerTest<cmr_msgs::srv::IsCollisionFree>
{
  public:

  CollisionCheckerServerMock(): cmr_tests_utils::BasicServiceServerTest<cmr_msgs::srv::IsCollisionFree>("collision_checker_server", "is_collision_free")
  {
    set_is_collision_free(false);
  }

  void set_is_collision_free(const bool & is_collision_free)
  {
    is_collision_free_ = is_collision_free;
  }

  void request_callback(const std::shared_ptr<rmw_request_id_t>, 
                        const std::shared_ptr<cmr_msgs::srv::IsCollisionFree::Request> request, 
                        std::shared_ptr<cmr_msgs::srv::IsCollisionFree::Response> response) override
  {
    cmr_tests_utils::BasicServiceServerTest<cmr_msgs::srv::IsCollisionFree>::last_request_ = request;

    response->is_collision_free = is_collision_free_;
    response->success = true;
  }

  private:

  bool is_collision_free_;

};

}

#endif