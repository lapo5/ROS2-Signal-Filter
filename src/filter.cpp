#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "Iir.h"

using namespace std::chrono_literals;

class FilterSignal : public rclcpp::Node {

  using Float32 = std_msgs::msg::Float32;

public: 
    enum FilteringTypes {
        Butterworth,
        Slerp,
        Median,
        NoFilter
    };

public:

    FilterSignal(const std::string node_name, rclcpp::NodeOptions options)
    : Node(node_name, options)
    {
        signal_raw_topic_ = this->declare_parameter("subscribers.raw_signal", "/signal/raw");
        
        pose_filter_topic_ = this->declare_parameter("publishers.filtered_signal", "/signal/filter");

        signal_filter_pub_ = this->create_publisher<Float32>(pose_filter_topic_, 1);

        signal_filtering_type = this->declare_parameter("filter.type", "butterworth");

        if (signal_filtering_type.compare("butterworth") == 0){
            signal_filtering_mode = FilteringTypes::Butterworth;
        }
        else if (signal_filtering_type.compare("slerp") == 0){
            signal_filtering_mode = FilteringTypes::Slerp;
        }
        else if (signal_filtering_type.compare("median") == 0){
            signal_filtering_mode = FilteringTypes::Median;
        }
        else if (signal_filtering_type.compare("no_filter") == 0){
            signal_filtering_mode = FilteringTypes::NoFilter;
        }
        else{
            std::cout << "[Signal Filter] Filter Type Undefined. Assuming NoFilter" << std::endl;
            signal_filtering_mode = FilteringTypes::NoFilter;
        }

        if (signal_filtering_mode == FilteringTypes::Median){
            
            N_measures = this->declare_parameter<int>("median_filter.n_samples", 10);
            std::cout << "[Signal Filter] Median Un-Implemented" << std::endl;
        }

        if (signal_filtering_mode == FilteringTypes::Slerp){
            N_measures = this->declare_parameter<int>("slerp_filter.n_samples", 10);
            std::cout << "[Signal Filter] Slerp Un-Implemented" << std::endl;
        }

        if (signal_filtering_mode == FilteringTypes::Median or signal_filtering_mode == FilteringTypes::Slerp){

            for(int i = 0; i < N_measures; i++){
                last_N_measures.push_back(0.0);
            }
            data_valid = false;
            counter = 0;
        }



        if (signal_filtering_mode == FilteringTypes::Butterworth){
            float sampling_rate = this->declare_parameter<float>("butterworth_filter.sampling_rate", 10.0);
            float cutoff_frequency = this->declare_parameter<float>("butterworth_filter.cutoff_frequency", 1.0);

            filter.setup(order, sampling_rate, cutoff_frequency);
            data_valid = true;
        }

        signal_raw_sub_ = this->create_subscription<Float32>(signal_raw_topic_, 1, 
            std::bind(&FilterSignal::pose_raw_callback_, this, std::placeholders::_1));

        std::cout << "[Signal Filter] Node Start" << std::endl;
    }

private:
    void pose_raw_callback_(const Float32::SharedPtr msg) {

        if (signal_filtering_mode == FilteringTypes::Median or signal_filtering_mode == FilteringTypes::Slerp)
        {
            last_N_measures[counter] = msg->data;

        }

        if(data_valid){

            Float32 filtered_signal;

            if (signal_filtering_mode == FilteringTypes::Butterworth){

                filtered_signal.data =  filter.filter(msg->data);
            }
            else if (signal_filtering_mode == FilteringTypes::NoFilter){

                filtered_signal.data =  msg->data;
            }

            signal_filter_pub_->publish(filtered_signal);

        }

        if (signal_filtering_mode == FilteringTypes::Median or signal_filtering_mode == FilteringTypes::Slerp){
            counter = (counter + 1) % N_measures;
            if (counter == 0)
            {
                data_valid = true;
            }
        }

    }

    std::string signal_raw_topic_;
    std::string pose_presence_topic_;
    std::string pose_filter_topic_;

    std::string signal_filtering_type = "butterworth";
    FilterSignal::FilteringTypes signal_filtering_mode;

    rclcpp::Publisher<Float32>::SharedPtr signal_filter_pub_{nullptr};
    rclcpp::Subscription<Float32>::SharedPtr signal_raw_sub_{nullptr};

    std::vector<double> last_N_measures;

    bool data_valid;

    int counter;
    int N_measures;

    static const int order = 3;

    Iir::Butterworth::LowPass<order> filter; 
};



int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    rclcpp::NodeOptions node_options;

    node_options.use_intra_process_comms(true);
    auto node = std::make_shared<FilterSignal>("signal_filter", node_options);

    executor->add_node(node);
    executor->spin(); 

    rclcpp::shutdown();

    return 0;
}
