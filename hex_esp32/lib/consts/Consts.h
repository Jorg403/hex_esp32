

const int __CMD_MAX_LEN__ = 256;  // Max command string length
const int __DELTA_TIME__ = 50;

//  array of arrays of servo positions
const int __SERVO_neg90_PWMs__[] = {
    140, 230, 250,  // arm 0
    125, 240, 210,  // arm 1
    130, 240, 200,  // arm 2
    100, 240, 150,  // arm 3
    140, 230, 140,  // arm 4
    110, 245, 120   // arm 5
};

const int __SERVO_0_PWMs__[] = {
    350, 470, 480,  // arm 0
    360, 490, 450,  // arm 1
    375, 485, 460,  // arm 2
    320, 485, 370,  // arm 3
    370, 470, 370,  // arm 4
    335, 485, 350   // arm 5
};

const int __SERVO_MIN_POS__[] = {-60, -150, -130};  // Servo min positions for each servo
const int __SERVO_MAX_POS__[] = {60, 30, 45};  // Servo max positions for each servo
