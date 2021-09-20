// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <math.h>
//#include "example.hpp"          // Include short list of convenience functions for rendering
#include <list>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <boost/format.hpp>
#include <algorithm>  
#include </home/tenderbook/rs_test/librealsense-master/examples/example-utils.hpp>
         
using namespace std;
using boost::format;
using boost::io::group;

double fr = 0.25;
double fr_plus = 0.45;
double pnt_delta = 0.002;



// Расстояние до экрана
// double d = -1.;

class Histogram {
    public:
        Histogram(int num, double min, double max, string fname_ ){
            num_of_cell = num;
            min_ = min;
            max_ = max;
            h = fabs(max_ - min_)/num;
            std::vector <int> hist(num);
            hist_ = hist; 
            file_name = fname_;
            
        }
        Histogram(int num, double min, double max){
            num_of_cell = num;
            min_ = min;
            max_ = max;
            h = fabs(max_ - min_)/num;
            std::vector <int> hist(num);
            hist_ = hist; 
            file_name = "hist.data";
            
        }
        void fill_hist(double a){
            if (a <= max_ && a >= min_){
                int i = static_cast<int> (fabs(a-min_)/h);
                try
                {
                    hist_[i]=hist_[i]+1;
                }
                catch(const std::exception& e)
                {
                    std::cerr << e.what() << '\n';
                }
                
            }

        }
        std::vector<int> get_hist(){
            return hist_;
        }
        void dump_to_file(double med, float sigma, int emitter, int accur, float laser_pwr, string title){
            fout.open(file_name, ios_base::trunc);

            fout << title << '\n';
            fout << "EMITTER  =" << emitter << '\n';
            fout << "VISUAL PRESET =" << accur << '\n';
            fout << "LASER POWER =" << laser_pwr << '\n';
            fout << "среднее = " << med << '\n';
            fout << "дисперсия =" << sigma << '\n';
            int i = 0;
            for (int a : hist_)
            {
                double z = min_+i*h;
                i++;
                fout << z << ", "<< a <<'\n';

            }
            
            fout.close();


        }

    private:
        double min_;
        double max_;
        int num_of_cell;
        double h;
        ofstream fout;
        std::string file_name;
        std::vector<int> hist_;


};


int main(int argc, char * argv[]) try
{
    double pi = 3.1415926535;
    double d = -1.;
    float LASER_PWR;
    int lk;
    double phi=0.;
    string serial;
    if (argc > 1){
        std::stringstream convert(argv[1]);
        convert >> d;
        d = -1.*d;

        std::istringstream convert1(argv[2]);
        convert1 >> lk;

        std::istringstream convert2(argv[3]);
        convert2 >> phi;
        phi = pi/180*phi;

       
        serial = static_cast<string> (argv[4]);




    }

    uint8_t EMITTER_OFF = 0;
    uint8_t EMITTER_ON = 1;
    
    ofstream dev_file;
    dev_file.open("options.info", ios_base::trunc);
    

    if (!device_with_streams({ RS2_STREAM_COLOR,RS2_STREAM_DEPTH }, serial))
    return EXIT_SUCCESS;

    string fn_in = (boost::format{"%s_%s.in"} % fabs(d) % (30*lk)).str();
    string fn_out = (boost::format{"%s_%s.out"} % fabs(d) % (30*lk)).str();
    string fn_point = (boost::format{"%s_%s.point"} % fabs(d) % (30*lk)).str();

    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);

    // cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    // cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);


        // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    // p.start();
    auto profile = p.start(cfg);
    // auto profile = p.start();

    auto sensor = profile.get_device().first<rs2::depth_sensor>();

    auto laser_range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
    LASER_PWR = laser_range.min + lk*laser_range.step;
    auto expl_range  = sensor.get_option_range(RS2_OPTION_EXPOSURE);
    // auto acc_range = sensor.get_option_range(RS2_OPTION_ACCURACY);
    auto options = sensor.get_supported_options();
    for (auto option : options){

        dev_file << "ВОЗМОЖНА OPTION : " << rs2_option_to_string(option) << '\n';
        
        try
        {
            auto current_val = sensor.get_option(option);
            dev_file << "current value is  : " << current_val << '\n';
            auto range = sensor.get_option_range(option);
            
            dev_file << "option is: " << rs2_option_to_string(option) << " def = " << range.def << '\n';
            dev_file << "option is: " << rs2_option_to_string(option) << " max = " << range.max << '\n';
            dev_file << "option is: " << rs2_option_to_string(option) << " min = " << range.min << '\n';
            dev_file << "option is: " << rs2_option_to_string(option) << " step = " << range.step << '\n';
    
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        

    }
    dev_file.close();

    // Set the device to High Accuracy preset of the D400 stereoscopic cameras
    // if (sensor && sensor.is<rs2::depth_stereo_sensor>())
    // {
    sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
    sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, EMITTER_OFF);
    sensor.set_option(RS2_OPTION_LASER_POWER, (laser_range.min+laser_range.step*lk));
    if (lk != 0){
        sensor.set_option(RS2_OPTION_EMITTER_ON_OFF, EMITTER_ON);
    }
        

    auto current_visual = sensor.get_option(RS2_OPTION_VISUAL_PRESET);
    std::cout << "current VISUAL PRESET is  : " << current_visual << '\n';
    auto current_emitter = sensor.get_option(RS2_OPTION_EMITTER_ON_OFF);
    std::cout << "current EMITTER is  : " << current_emitter << '\n';
    auto laser_pwr = sensor.get_option(RS2_OPTION_LASER_POWER);
    std::cout << "current laser power is  : " << laser_pwr << '\n';

    // }


    static Histogram histogram_in = {400, 0.0, 2.0, fn_in};
    static Histogram histogram_out = {400, 0.0, 2.0, fn_out};
    static Histogram histogram_point = {400, 0.0, 2.0, fn_point};

    while (true)
    {
        try
        {
            // Block program until frames arrive
        rs2::frameset frames = p.wait_for_frames();

        // Try to get a frame of a depth image
        rs2::depth_frame depth = frames.get_depth_frame();

        // Get the depth frame's dimensions
        auto width = depth.get_width();
        auto height = depth.get_height();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        // get vertices
        auto vertices = points.get_vertices();
        auto tex_coords = points.get_texture_coordinates(); // and texture coordinates

        auto size = points.size();

        

        double k=0;
        double mz = 0.0;
        std::list<double> d_list = {};
        double sig_0 = 0.0;
        double mdz = 0.0; 
        
        double k_1=0;
        double mz_1 = 0.0;      
        std::list<double> d_list_1 = {};
        double sig_0_1 = 0.0;
        double mdz_1 = 0.0; 

        double k_2=0;
        double mz_2 = 0.0;
        std::list<double> d_list_2 = {};
        double sig_0_2 = 0.0;
        double mdz_2 = 0.0; 

        ofstream vert_file;
        vert_file.open("vertex.data", ios_base::trunc);

        for (int i = 0; i < points.size(); i++)
        {
            if (vertices[i].z)
            {   
                
                // upload the point and texture coordinates only for points we have depth data for
                auto v = vertices[i];
                double x=v.x;

                if(phi !=0.0){
                    x=v.x*cos(phi)-v.z*sin(phi);
                }

                if (x <= fr && x >= (-1*fr) && v.y >= (-1*fr) && v.y <= 0.2 ){
                    auto coord = tex_coords[i];
                    vert_file << v.z << '\n';
                    // if (phi == 0.0)
                    // {
                        double dz = v.z + d;
                        mdz = mdz + dz;
                        sig_0 = sig_0 + dz*dz;
                        // d_list.push_back(dz);
                        d_list.push_back(v.z);
                        histogram_in.fill_hist(v.z);
                        // mz = mz + dz;
                        mz= mz + v.z;

                    // } else{
                    //     double z = v.z;
                    //     z=z*cos(phi)+v.x*sin(phi);
                    //     double dz = z + d;
                    //     mdz = mdz + dz;
                    //     sig_0 = sig_0 + dz*dz;
                    //     // d_list.push_back(dz);
                    //     d_list.push_back(z);
                    //     histogram_in.fill_hist(z);
                    //     // mz = mz + dz;
                    //     mz= mz + z;
                    // }                  
                    
                    k++;
                }else if ((fabs(v.x) > fr && fabs(v.x) < fr_plus && v.y >= (-1*fr) && v.y <= 0.2) || (v.y < (-1*fr) && v.y >= (-1*fr_plus) && fabs(v.x) < fr_plus)){
                    auto coord = tex_coords[i];
                    double dz = v.z + d;
                    mdz_1 = mdz_1 + dz;
                    sig_0_1 = sig_0_1 + dz*dz;
                    // d_list.push_back(dz);
                    d_list_1.push_back(v.z);
                    histogram_out.fill_hist(v.z);
                    // mz = mz + dz;
                    mz_1= mz_1+ v.z;

                    k_1++;
                } 

                if (fabs(v.y)<=pnt_delta && fabs(v.x)<=(fr+pnt_delta) && fabs(v.x) >= (fr-pnt_delta))
                {

                    auto coord = tex_coords[i];
                    double dz = v.z + d;
                    mdz_2 = mdz_2 + dz;
                    sig_0_2 = sig_0_2 + dz*dz;
                    // d_list.push_back(dz);
                    d_list_2.push_back(v.z);
                    histogram_point.fill_hist(v.z);
                    // mz = mz + dz;
                    mz_2= mz_2+ v.z;

                    k_2++;
                    
                }
                
            }
        }

        vert_file.close();

        sig_0 = sqrt(sig_0/(k-1));
        
        mz = mz / k;
        mdz= mdz/k;
        double sig = 0.0;
        for (double dd : d_list){
            sig = sig + (mz - dd)*(mz - dd);
        }
        d_list.clear();
        sig = sqrt(sig/(k-1));

        sig_0_1 = sqrt(sig_0_1/(k_1-1));
        mz_1 = mz_1 / k_1;
        mdz_1= mdz_1/ k_1;
        double sig_1 = 0.0;
        for (double dd : d_list_1){
            sig_1 = sig_1 + (mz_1 - dd)*(mz_1 - dd);
        }
        d_list_1.clear();
        sig_1 = sqrt(sig_1/((k_1-1)));

        sig_0_2 = sqrt(sig_0_2/(k_2*(k_2-1)));
        mz_2 = mz_2 / k_2;
        mdz_2= mdz_2/ k_2;
        double sig_2 = 0.0;
        for (double dd : d_list_2){
            sig_2= sig_2 + (mz_2 - dd)*(mz_2 - dd);
        }
        d_list_2.clear();
        sig_2 = sqrt(sig_2/(k_2-1));



        // Query the distance from the camera to the object in the center of the image
        float dist_to_center = depth.get_distance(width / 2, height / 2);
        auto H_in = histogram_in.get_hist();
        
        std::string title = (boost::format{"для внутренней области, расстояние d = %s"} % d).str();
        histogram_in.dump_to_file(mz, sig, current_emitter, current_visual, laser_pwr, title);
        histogram_out.dump_to_file(mz_1, sig_1, current_emitter, current_visual, laser_pwr, (boost::format{"для внешней области, расстояние d = %s"} % d).str());

        histogram_point.dump_to_file(mz_2, sig_2, current_emitter, current_visual, laser_pwr, (boost::format{"для мусора, расстояние d = %s"} % d).str());

        // Print the distance
        std::cout << "The camera is facing an object " << dist_to_center << " meters away " << "Number of vertices is " << k << "sigma = "<< sig_0 <<"\r";
       
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
