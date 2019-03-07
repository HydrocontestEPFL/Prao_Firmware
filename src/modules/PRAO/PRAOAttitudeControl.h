//
// Created by Johan on 01.03.2019.
//

#ifndef FIRMWARE_PRAOATTITUDECONTROL_H
#define FIRMWARE_PRAOATTITUDECONTROL_H


class PRAOAttitudeControl {
    public:

private:
    //Initialise la structure de parametres
    struct {
        float yaw_p;
        float pitch_p;
    } _params{};

    //Initialise la structure des handles de param
    struct {
        param_t yaw_p;
    } _param_handles{};
};


#endif //FIRMWARE_PRAOATTITUDECONTROL_H
