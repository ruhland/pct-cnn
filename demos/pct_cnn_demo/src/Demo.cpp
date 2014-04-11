#include <boost/program_options.hpp>
#include <iostream>
#include <exception>

#include "Demo.hpp"

Demo::Demo(){
   std::cout<<"Demo created";
}

void Demo::setInputFile(const std::string& name){};
void Demo::setOutputFile(const std::string& output){};
void Demo::enableKinect(){};
void Demo::run(){};

int main (int argc, char **argv)
    {
    try{
    // Parsing input parameters:
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("i",boost::program_options::value<std::string>()->required(),"input file (required)")
        ("o",boost::program_options::value<std::string>(),"output file")
        ("k","enable Kinect")
    ;

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);

     if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    boost::program_options::notify(vm);    
    
    Demo demo;
    if(vm.count("k"))
        demo.enableKinect();
    demo.setInputFile(vm["i"].as<std::string>());
    if(vm.count("o"))
        demo.setOutputFile(vm["o"].as<std::string>());
    demo.run();
    }
    /*catch(boost::program_options::error& e){
            std::cout<<"Exception in boost :"<<e.what();
    }*/
    catch(const std::exception& e){
        std::cout<<"Exception in programm:"<<e.what()<<std::endl;    
    }
     
}
