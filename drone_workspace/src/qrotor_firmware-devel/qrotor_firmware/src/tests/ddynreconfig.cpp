#include <ros/ros.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <string>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

class DDynamicReconfigureExample {
  private:
    ros::NodeHandle& nh_;
    //    std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec;
    ddynamic_reconfigure::DDynamicReconfigure ddynrec;
    double param_1;
    //    std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> ddynrec_container_;
  public:
    DDynamicReconfigureExample (ros::NodeHandle& _nh): nh_(_nh) {


    }

    void init() {
        // Now parameter can be modified from the dynamic_reconfigure GUI or other tools and the callback is called on each update
        //        std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh_);
        //        readAndSetDynamicParam(nh_, ddynrec, "param_1", 0, 100);
        ddynrec.registerVariable<double>("param_1", 10.0, [this](double new_value) {
            param_1 = new_value;
            ROS_INFO("param updated");
        }, "parameter description", -100, 100);
        //        readAndSetDynamicParam(nh_, ddynrec, "right", 0, 100);
        //        readAndSetDynamicParam(nh_, ddynrec, "top", 0, 100);
        //        readAndSetDynamicParam(nh_, ddynrec, "bottom", 0, 100);
        ddynrec.publishServicesTopics();
        //        ddynrec_container_.push_back(ddynrec);
    }

    void run() {
        ROS_INFO("param_1: %f", param_1);
    }


};


int main (int argc, char** argv) {
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;

    DDynamicReconfigureExample dr_example_(nh);
    dr_example_.init();

    ros::Rate loop_rate(10);
    while(nh.ok()) {
        dr_example_.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}

/******************************************************************************************************************************/
//    void readAndSetDynamicParam(ros::NodeHandle& _nh, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec,
//                                const std::string option_name, const int min_val, const int max_val) {
//        _nh.param(option_name, *option_value, *option_value); //param (const std::string &param_name, T &param_val, const T &default_val) const
//        if (*option_value < min_val) *option_value = min_val;
//        if (*option_value > max_val) *option_value = max_val;

//        ddynrec->registerVariable<int>( option_name, *option_value,
//        [this, sensor, option_name](int new_value) {
//            set_auto_exposure_roi(option_name, sensor, new_value);
//        },
//        option_name, min_val, max_val);
//    }
//struct Parameters {
//    Parameters() {}

//    int int_test;
//    bool bool_test;
//    std::string string_test;
//};

//void paramCb(int new_value) {
//    global_int = new_value;
//    ROS_INFO("Param modified");
//}

//void setParameters( ) {

//}

//void readAndSetDynamicParam(ros::NodeHandle& nh1, std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec,
//                            const std::string option_name, const int min_val, const int max_val) {
//    nh1.param(option_name, *option_value, *option_value); //param (const std::string &param_name, T &param_val, const T &default_val) const
//    if (*option_value < min_val) *option_value = min_val;
//    if (*option_value > max_val) *option_value = max_val;

//    ddynrec->registerVariable<int>( option_name, *option_value,
//    [this, sensor, option_name](int new_value) {
//        set_auto_exposure_roi(option_name, sensor, new_value);
//    },
//    option_name, min_val, max_val);
//}

//int main(int argc, char** argv) {
//    // ROS init stage
//    ros::init(argc, argv, "ddynamic_tutorials");
//    ros::NodeHandle nh;
//    //    ddynamic_reconfigure::DDynamicReconfigure ddr;

//    //    ddr.registerVariable<int>("params", 10 /* initial value */, boost::bind(paramCb, _1), "param description", -10, 20);
//    //    ddr.publishServicesTopics();

//    std::vector<std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure>> _ddynrec;
//    // Now parameter can be modified from the dynamic_reconfigure GUI or other tools and the callback is called on each update
//    std::shared_ptr<ddynamic_reconfigure::DDynamicReconfigure> ddynrec = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(nh1);

//    readAndSetDynamicParam(nh, ddynrec, "left", 0, 100);
//    readAndSetDynamicParam(nh, ddynrec, "right", 0, 100);
//    readAndSetDynamicParam(nh, ddynrec, "top", 0, 100);
//    readAndSetDynamicParam(nh, ddynrec, "bottom", 0, 100);

//    ddynrec->publishServicesTopics();
//    _ddynrec.push_back(ddynrec);

//    ros::spin();
//    return 0;
//}

//#include <ros/ros.h>
//#include <ddynamic_reconfigure/ddynamic_reconfigure.h>

//using namespace ddynamic_reconfigure;

///**
//  Topics:
//  * /dynamic_tutorials/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
//  * /dynamic_tutorials/parameter_updates [dynamic_reconfigure/Config]
//  Services:
//  * /dynamic_tutorials/set_parameter:  dynamic_reconfigure/Reconfigure
//*/
//int int_test = 0;
//void paramCb(int new_value) {
//    int_test = new_value;
//    ROS_INFO("Param modified");
//}

//int main(int argc, char** argv) {
//    ros::init(argc, argv, "fake_dynamic_reconfigure");

//    ros::NodeHandle nh("fake_dyanmic_reconfigure");

//    double double_test = 0.0;
//    double double_range = 2;
//    bool bool_test = false;
//    double changing_variable = 0.0;
//    std::string str_test = "";
//    DDynamicReconfigure ddr(nh);
//    DDynamicReconfigure ddr2(ros::NodeHandle(nh, "nh2"));
//    DDynamicReconfigure ddr3(ros::NodeHandle(nh, "nh3"));

//    ddr.RegisterVariable(&double_test, "double_test");
//    ddr.RegisterVariable(&double_range, "double_range_test", 0, 10);
//    ddr.RegisterVariable(boost::bind(paramCb, _1), "int_test", -50, 50);
//    ddr.RegisterVariable(&bool_test, "bool_test");
//    ddr.registerVariable("str_test", &str_test);
//    ddr.RegisterVariable(&changing_variable, "changing_variable");

//    std::map<std::string, int> enum_map = {{"ZERO", 0}, {"ONE", 1}, {"ONE_HUNDRED", 100}};
//    ddr.registerEnumVariable<int>("enum_int", enum_map["ONE"], [](int new_value) {
//        ROS_INFO_STREAM("Value changed to " << new_value);
//    }, "Enum parameter", enum_map, "enum description");

//    std::map<std::string, std::string> str_enum_map = {{"ZERO", "zero"}, {"ONE", "one"}, {"ONE_HUNDRED", "one hundred"}};
//    ddr.registerEnumVariable<std::string>("enum_string", str_enum_map["ONE"], [](std::string new_value) {
//        ROS_INFO_STREAM("Value changed to " << new_value);
//    }, "Enum parameter", str_enum_map, "enum description");

//    std::map<std::string, double> double_enum_map = {{"ZERO", 0.0}, {"ONE", 1.1}, {"ONE_HUNDRED", 100.001}};
//    ddr.registerEnumVariable<double>("enum_double", double_enum_map["ONE"], [](double new_value) {
//        ROS_INFO_STREAM("Value changed to " << new_value);
//    }, "Enum parameter", double_enum_map, "enum description");


//    std::map<std::string, bool> bool_enum_map = {{"false", false}, {"true", true}, {"also true", true}};
//    ddr.registerEnumVariable<bool>("enum_bool", bool_enum_map["ONE"], [](bool new_value) {
//        ROS_INFO_STREAM("Value changed to " << new_value);
//    }, "Enum parameter", bool_enum_map, "enum description");

//    ddr2.RegisterVariable(&double_test, "double_test");
//    ddr2.RegisterVariable(&int_test, "int_test");
//    ddr2.RegisterVariable(&bool_test, "bool_test");

//    ddr.PublishServicesTopics();
//    ddr2.PublishServicesTopics();
//    ddr3.PublishServicesTopics();


//    ROS_INFO("Spinning node");

//    while (nh.ok()) {
//        changing_variable += 0.5;
//        std::cerr << "changing_variable " << changing_variable << std::endl;
//        std::cerr << "double " << double_test << std::endl;
//        std::cerr << "double range" << double_range << std::endl;
//        std::cerr << "int " << int_test << std::endl;
//        std::cerr << "bool " << bool_test << std::endl;
//        std::cerr << "str " << str_test << std::endl;
//        std::cerr << "*********" << std::endl;
//        ros::spinOnce();
//        ros::Duration(1.0).sleep();
//    }
//    return 0;
//}
