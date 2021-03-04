#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>

#include <add.h>

int main(int argc, char *argv[])
{
    // parse my-info.json
    std::ifstream file(CMM_MY_INFO);
    auto json = nlohmann::json::parse(file);
    file.close();
    std::cout << "full name : " << json["full name"] << std::endl;
    std::cout << "student id: " << json["student id"] << std::endl;
    std::cout << "nethz name: " << json["nethz name"] << std::endl;

    // test math::add
    bool testPassed = (math::add({2,3}, {1,2}) == Eigen::Vector2f{3,5});
    return (testPassed) ? 0 : 1;
}
