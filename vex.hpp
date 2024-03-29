#include <iostream>
#include <math.h>
#include <cstdint>
#include <vector>
#include <string>
#include <cctype>
#include <fstream>
#include "json.hpp"
#include <unordered_map>
namespace vex{
    enum debugMode{
        control,
        distance,
        brain,
        gyro,
        def 
    };
    enum currentUnits{
        amp
    };
    enum torqueUnits{
        Nm = 21,
        InLb = 185
    };
    enum gearSetting{
        ratio36_1 = 36,
        ratio18_1 = 18,
        ratio6_1 = 6
    };
    enum velocityUnits{
        pct,
        rpm = 6,
        dps = 1
    };
    enum brakeType{
        coast,
        brake,
        hold
    };
    enum rotationUnits{
        degrees = 1,
        rev = 360
    };
    enum timeUnits{
        sec = 1000,
        msec = 1
    };
    enum directionType{
        forward = 1,
        reverse = -1
    }; 
    enum controllerType{
        primary,
        partner
    };
    class motor;
    std::vector<motor*> motorList;
    class motor{
        private:
            std::int32_t port;
            directionType revers = forward;
            gearSetting gears = ratio18_1;
            double maxTourqe = 100;
            brakeType stopping = coast;
            rotationUnits rotationValUnit = degrees;
            rotationUnits positionValUnit = degrees;
            std::int32_t timeout;
            timeUnits timeoutUnits;
        public:
            bool brake = false;
            double lastPosition = 0;
            double lastRotation = 0;
            double rotationVal = 0;
            double positionVal = 0;
            double velocityVel = 0;
            bool spinning = false;
            motor(std::int32_t port):port(port){motorList.push_back(this);};
            motor(std::int32_t port,bool reversIn):port(port){
                motorList.push_back(this);
                revers = (reversIn)?reverse:forward;
            };
            motor(std::int32_t port,gearSetting gears):port(port),gears(gears){motorList.push_back(this);};
            motor(std::int32_t port,gearSetting gears,bool reversIn):port(port),gears(gears){
                motorList.push_back(this);
                revers = (reversIn)?reverse:forward;
            };
            bool installed(){
                return true;
            }
            void setReversed(bool value){
                revers = (value)?reverse:forward;
            }
            void setVelocity(double vel, velocityUnits units){
                if(units != pct){
                    velocityVel = (vel/units)/60;
                }else{
                    velocityVel = (vel/100)*60;
                }
            }
            void setBrake(brakeType mode){
                stopping = mode;
                brake = true;
            }
            void setStopping(brakeType mode){
                stopping = mode;
            }
            void resetRotation(){
                rotationVal = 0;
            }
            void resetPosition(){
                positionVal = 0;
            }
            void setRotation(double value, velocityUnits units = pct){
                rotationVal = value;
            }
            void setRotation(double value, rotationUnits units){
                rotationVal = value;
                rotationValUnit = units;
            }
            void setPosition(double value, velocityUnits units = pct){
                positionVal = value;
            }
            void setPosition(double value, rotationUnits units){
                positionVal = value;
                positionValUnit = units;
            }
            void setTimeout(std::int32_t time, timeUnits units){
                timeout = time;
                timeoutUnits = units;
            }
            void spin(directionType dir,bool debug = false){
                if(!brake){
                    lastRotation = rotationVal;
                    lastPosition = positionVal;
                    if(dir*revers > 0){
                        rotationVal += (velocityVel/gears);
                        positionVal += (velocityVel/gears);
                    }else{
                        rotationVal -= (velocityVel/gears);
                        positionVal -= (velocityVel/gears);
                    }
                    if(debug){
                        std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                        std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                    }
                }else -brake;
            }
            void spin(directionType dir, double vel, velocityUnits units,bool debug = false){//debug: logs position and rotation
                if(!brake){
                    double finVelocity;
                    if(units != pct)finVelocity = (vel/units)/60;
                    else finVelocity = (vel/100)*60;
                    lastRotation = rotationVal;
                    lastPosition = positionVal;
                    if(dir*revers > 0){
                        rotationVal += (finVelocity/gears);
                        positionVal += (finVelocity/gears);
                    }else{
                        rotationVal -= (finVelocity/gears);
                        positionVal -= (finVelocity/gears);
                    }
                    if(debug){
                        std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                        std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                    }
                }else -brake;
            }
            bool spinTo(double rot, rotationUnits units, double vel, velocityUnits units_v, bool waitForCompletion = true, bool debug = false){//debug: logs position and rotation
                if(rot*units != rotationVal){
                    if(!brake){
                        spinning = true;
                        double finVelocity;
                        if(units_v != pct)finVelocity = (vel/units_v)/60;
                        else finVelocity = (vel/100)*60;
                        lastRotation = rotationVal;
                        lastPosition = positionVal;
                        if(rotationVal-(rot*units) > 0){
                            rotationVal += (finVelocity/gears);
                            positionVal += (finVelocity/gears);
                        }else{
                            rotationVal -= (finVelocity/gears);
                            positionVal -= (finVelocity/gears);
                        }
                        if(waitForCompletion){
                            rotationVal = rot;
                            positionVal += (rot*units)-rotationVal;
                            std::cout << "Waited for spinTo comletion"<< std::endl;
                        }
                        if(debug){
                            std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                            std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                        }
                        if(rot*units == rotationVal){
                            return true;
                            spinning = false;
                        }
                        else return false;
                    }
                }else{
                    spinning = false;
                    if(debug){
                        std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                        std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                    }
                    return true;
                }
            }
            bool spinToPostion(double rot, rotationUnits units, double vel, velocityUnits units_v, bool waitForCompletion = true, bool debug = false){//debug: logs possition and rotationVal
                if(rot*units != rotationVal){
                    if(!brake){
                        spinning = true;
                        double finVelocity;
                        if(units_v != pct)finVelocity = (vel/units_v)/60;
                        else finVelocity = (vel/100)*60;
                        lastRotation = rotationVal;
                        lastPosition = positionVal;
                        if(rotationVal-(rot*units) > 0){
                            rotationVal += (finVelocity/gears);
                            positionVal += (finVelocity/gears);
                        }else{
                            rotationVal -= (finVelocity/gears);
                            positionVal -= (finVelocity/gears);
                        }
                        if(waitForCompletion){
                            rotationVal = rot;
                            positionVal += (rot*units)-rotationVal;
                            std::cout << "Waited for spinToPosition comletion"<< std::endl;
                        }
                        if(debug){
                            std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                            std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                        }
                        if(rot*units == rotationVal){
                            return true;
                            spinning = false;
                        }
                        else return false;
                    }
                }else{
                    spinning = false;
                    if(debug){
                        std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                        std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                    }
                    return true;
                }
            }
            bool spinTo(double rot, rotationUnits units, bool waitForCompletion = true, bool debug = false){//debug: logs possition and rotationVal
                if(rot*units != rotationVal){
                    if(!brake){
                        spinning = true;
                        lastRotation = rotationVal;
                        lastPosition = positionVal;
                        if(rotationVal-(rot*units) > 0){
                            rotationVal += (velocityVel/gears);
                            positionVal += (velocityVel/gears);
                        }else{
                            rotationVal -= (velocityVel/gears);
                            positionVal -= (velocityVel/gears);
                        }
                        if(waitForCompletion){
                            rotationVal = rot;
                            positionVal += (rot*units)-rotationVal;
                            std::cout << "Waited for spinTo completion"<< std::endl;
                        }
                        if(debug){
                            std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                            std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                        }
                        if(rot*units == rotationVal){
                            return true;
                            spinning = false;
                        }
                        else return false;
                    }
                }else{
                    spinning = false;
                    if(debug){
                        std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                        std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                    }
                    return true;
                }
            }
            bool spinToPosition(double rot, rotationUnits units, bool waitForCompletion = true, bool debug = false){//debug: logs possition and rotationVal
                if(rot*units != rotationVal){
                    if(!brake){
                        spinning = true;
                        lastRotation = rotationVal;
                        lastPosition = positionVal;
                        if(rotationVal-(rot*units) > 0){
                            rotationVal += (velocityVel/gears);
                            positionVal += (velocityVel/gears);
                        }else{
                            rotationVal -= (velocityVel/gears);
                            positionVal -= (velocityVel/gears);
                        }
                        if(waitForCompletion){
                            rotationVal = rot;
                            positionVal += (rot*units)-rotationVal;
                            std::cout << "Waited for spinToPosition completion"<< std::endl;
                        }
                        if(debug){
                            std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                            std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                        }
                        if(rot*units == rotationVal){
                            return true;
                            spinning = false;
                        }
                        else return false;
                    }
                }else{
                    spinning = false;
                    if(debug){
                        std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                        std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                    }
                    return true;
                }
            }
            bool spinFor(directionType dir,double rot, rotationUnits units, double vel, velocityUnits units_v, bool waitForCompletion=true, bool debug = false){//debug: logs possition and rotationVal
                std::cout << "Stop using spinFor, just use spin\n This is the last time I'm changing the rotation and position for you" << std::endl;
                lastPosition = positionVal;
                lastRotation = rotationVal;
                rotationVal += (rot*units)-rotationVal;
                positionVal += (rot*units)-rotationVal;
                if(debug){
                    std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                    std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                }
                return true;
            }
            bool spinFor(double rot, rotationUnits units,double vel = -1.11,velocityUnits units_v = rpm,bool waitForCompletion = true, bool debug = false){//debug: logs possition and rotationVal
                if(vel == -1.11)vel = velocityVel;
                std::cout << "Stop using spinFor, just use spin\n This is the last time I'm changing the rotation and position for you" << std::endl;
                lastPosition = positionVal;
                lastRotation = rotationVal;
                rotationVal += (rot*units)-rotationVal;
                positionVal += (rot*units)-rotationVal;
                if(debug){
                    std::cout << "Rotation: " << rotationVal*rotationValUnit << std::endl;
                    std::cout << "Position: " << positionVal*positionValUnit << std::endl;
                }
                return true;
            }
            bool isSpinning(){return spinning;}
            bool isDone(){return -spinning;}
            //isSpinningMode
            void stop(brakeType mode = coast){brake = true;}
            void setMaxTorque(double value,rotationUnits units){maxTourqe = value;}
            void setMaxTorque(double value,torqueUnits units){maxTourqe = ((value*10)/units)*100;}
            void setMaxTorque(double value,currentUnits units){maxTourqe = (value/1.2)*100;}
            directionType direction(){
                if(positionVal-lastPosition<0)return reverse; 
                else return forward;

            }
            double rotation(rotationUnits units){return rotationVal/units;}
            double position(rotationUnits units){return positionVal/units;}
            double velocity(velocityUnits units){
                if(units != pct) return (velocityVel*60*units)/gears;
                else return (velocityVel/-60)*100;
            }
            double current(currentUnits units = amp){return revers*1.2;}
            double current(rotationUnits units){return revers*100;}
    };
    class controller;
    std::vector<controller*> controllerList;
    class controller{
        private:
            controllerType type = primary;
        public:
            controller(controllerType id):type(id){
                controllerList.push_back(this);
            };
            controller():type(primary){
                controllerList.push_back(this);
            };
            class button{
                public:
                    bool pressedDown = false;
                    bool pressedNow = false;
                    bool releasedNow = false;
                    void pressed(void(*callback)(void),bool debug = false){
                        if(pressedNow){
                            if(debug)std::cout << "Button pressed" << std::endl;
                            callback();
                            pressedNow = false;
                            pressedDown = true;
                        }
                    }
                    bool pressing(){
                        if(pressedDown)return true;
                        else return false;
                    }

            };
            button ButtonL1;
            button ButtonL2;
            button ButtonR1;
            button ButtonR2;
            button ButtonUp;
            button ButtonDown;
            button ButtonLeft;
            button ButtonRight;
            button ButtonX;
            button ButtonB;
            button ButtonY;
            button ButtonA;
            std::unordered_map<std::string,button*> buttonMap{
                {"a",&ButtonA},
                {"b",&ButtonB},
                {"x",&ButtonX},
                {"y",&ButtonY},//
                {"up",&ButtonUp},
                {"down",&ButtonDown},
                {"left",&ButtonLeft},
                {"right",&ButtonRight},//
                {"l1",&ButtonL1},
                {"l2",&ButtonL2},
                {"r1",&ButtonR1},
                {"r2",&ButtonR2}//
            };
            class axis{
                public:
                    int32_t axisValue = 0;
                    int32_t lastValue = 0;
                    void changed(void(*callback)(void)){
                        if(axisValue != lastValue){
                            callback();
                        }
                    }
                    int32_t positionVal(velocityUnits units = velocityUnits::pct){
                        return axisValue;
                    }
                    int32_t value(velocityUnits units = velocityUnits::pct){
                        return axisValue;
                    }
            };
            axis Axis1;//Right side to side
            axis Axis2;//Right up and down
            axis Axis3;//Left up and down
            axis Axis4;//Left side to side
            std::unordered_map<std::string,axis*> axisMap{
                {"a1",&Axis1},
                {"a2",&Axis2},
                {"a3",&Axis3},
                {"a4",&Axis4}
            };
            class lcd{
                private:
                    int32_t cursorRow = 1;
                    int32_t cursorColumn = 1;
                    std::string rows[3] = {"","",""};
                public:
                    void setCursor(int32_t row, int32_t col){
                        cursorRow = row;
                        cursorColumn = col;
                    }
                    int32_t column(){return cursorColumn;}
                    int32_t row(){return cursorRow;}
                    template<typename T>
                    void print(T value){
                        if(cursorColumn == 1){
                            rows[cursorRow-1] = "";
                        }
                        rows[cursorRow-1].append(std::to_string(value));
                        std::cout << rows[0] << "\n" << rows[1] << "\n" << rows[2] << std::endl;
                    }
                    void clearScreen(){
                        rows[0] = "";
                        rows[1] = "";
                        rows[2] = "";
                    }
                    void clearLine(int number = -1){
                        if(number == -1) number = static_cast<int>(cursorRow);
                        rows[number-1] = "";
                    }
                    void newLine(){
                        if(cursorRow != 3){cursorRow++;}
                    }

            };
            lcd Screen;
            bool installed(){return true;}
            void rumble(const char *str){
                std::cout << "The controller rumbles: " << str << std::endl;
            }
    };
    class debuger{
    private:
        int length = 0;
        int count = 0;
        struct Event{
            int second;
            std::string event;
            int eventValue;
            Event(int second, std::string event,int eventValue): second(second),event(event),eventValue(eventValue){};
        };
        std::vector<Event> timeline;
    public:
        void debug(){
            //manual setup            
            if(count == 0){
                std::ifstream setup;
                setup.open("./setup.json");
                std::string ans = "n";
                if(setup){
                    std::cout << "\nSetup file found in directory. Load it? [y/n]";
                    std::cin >> ans;
                    if(ans == "y"){
                        std::ifstream read("./setup.json");
                        nlohmann::json json = nlohmann::json::parse(read);

                        for(auto& [key,value] : json.items()){
                            if(key != "length"){
                                for(auto& [ikey,ivalue] : value.items()){
                                    timeline.push_back(Event(std::stoi(key),ikey,ivalue));
                                    std::cout << "\nValue loaded: " << std::stoi(key)<<" , "<< ikey<< " , "<<ivalue;
                                }
                            }else{
                                length = value;
                                std::cout << "\nValue loaded: length , " << value;
                            }
                        }
                    }
                }
                if(ans != "y"){
                    std::cout << "How long will this test run for (in seconds)\n>";
                    std::cin >> length;
                    int exit = 0;
                    while(exit == 0){
                        if(!controllerList.empty()){
                            std::string button = "none";
                            std::cout << "\nSelect button or axis for setup (no uppercase)\n>";
                            std::cin >> button;
                            if(button == "exit" || button == "e"){
                                exit = 1;
                                continue;
                            }
                            if(button == "a1" || button == "a2" || button == "a3" || button == "a4"){
                                while(true){
                                    int second = 0;
                                    std::cout << "\nWhen will you like this to occure\n>";
                                    std::cin >> second;
                                    if(second == -1)break;
                                    if(second<0||second>length)std::cout << "\nActivation out of range";
                                    int eventId;
                                    std::cout << "\nWhat value is the axis moving to (-100 to 100)\n>";
                                    std::cin >> eventId;
                                    if(eventId>100 || eventId <-100){
                                        std::cout << "\nValue out of range";
                                        continue;
                                    }
                                    timeline.push_back(Event(second,button,eventId));
                                    std::cout << "\nEvent recorded, type '-1' to exit axis setup";
                                }
                            }else if(controllerList.at(0)->buttonMap.find(button) != controllerList.at(0)->buttonMap.end()){
                                while(true){
                                    int second = -1;
                                    std::cout << "\nWhen will you like this to event to occure\n>";
                                    std::cin >> second;
                                    if(second == -1)break;
                                    if(second<0||second>length)std::cout << "\nActivation out of range";
                                    int eventId = -1;
                                    std::cout << "\n1 = activate, 0 = deactivate\n>";
                                    std::cin >> eventId;
                                    if(eventId !=0 && eventId != 1){
                                        std::cout << "\nInvalid event Id";
                                        continue;
                                    }
                                    timeline.push_back(Event(second,button,eventId));
                                    std::cout << "\nEvent recorded, type '-1' to exit button setup";
                                }
                            }else{
                                std::cout << "\nInvalid button/axis ID";
                                continue;
                            }
                            std::cout << "\nType 'exit' to end controller setup";
                        }else exit = 1;
                    }
                    nlohmann::json jsonData;
                    jsonData["length"] = length;
                    for(int i=0; i<timeline.size();i++){
                        jsonData[std::to_string(timeline.at(i).second)][timeline.at(i).event] = timeline.at(i).eventValue;
                    }
                    std::ofstream save("setup.json");
                    save << jsonData;
                }
            }
            if(count > length)exit(0);
            //do events
            std::cout << "\nEvents:";
            for(int i = 0; i<timeline.size();i++){
                if(timeline.at(i).second == count){
                    if(controllerList.at(0)->buttonMap.find(timeline.at(i).event) != controllerList.at(0)->buttonMap.end()){ 
                        controllerList.at(0)->buttonMap.at(timeline.at(i).event)->pressedDown = (bool)timeline.at(i).eventValue;
                        controllerList.at(0)->buttonMap.at(timeline.at(i).event)->pressedNow = (bool)timeline.at(i).eventValue;
                    }else{controllerList.at(0)->axisMap.at(timeline.at(i).event) -> axisValue = timeline.at(i).eventValue;}
                    std::cout << "\n  " << timeline.at(i).event << " to value " << timeline.at(i).eventValue;
                }
            }
            std::cout << "\nMotors:";
            for(int i=0;i<motorList.size();i++){
                std::cout << "\nMotor " << i+1;
                std::cout << "\n  Rotation = " << motorList.at(i) -> rotationVal;
                std::cout << "\n  Position = " << motorList.at(i) -> positionVal;
                std::cout << "\n  Velocity = " << motorList.at(i) -> velocityVel;
                if(motorList.at(i) -> rotationVal < motorList.at(i) -> lastRotation)std::cout << "\n  Moved in the negative direction";
                else if(motorList.at(i) -> rotationVal > motorList.at(i) -> lastRotation)std::cout << "\n  Moved in the positive direction";
                else std::cout << "\n  Did not move";
                motorList.at(i) -> brake = false;
            }
            std::string nothing = "";
            std::cout << "\n Second: " << count << "\nPress enter to continue and ctr+c to end";
            std::cin >> nothing;
            for(int i = 0;i<motorList.size();i++){
                motorList.at(i)->lastPosition = motorList.at(i)->position(degrees);
                motorList.at(i)->lastRotation = motorList.at(i)->rotation(degrees);
            }
            count++;
        }
    };
    debuger Debug;
}
