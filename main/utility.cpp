int mod(int a, int b) {
    return (a % b + b) % b;
}

/* diff_angles(350, 10);   // => -20 */
/* diff_angles(10, 350);   // =>  20 */
/* diff_angles(-170, 170); // =>  20 */
/* diff_angles(-170, 190); // =>   0 */
int diff_angles(int a, int b) {
    int diff = a - b;
    return mod((diff + 180), 360) - 180;
} 

float diff_anglesf(float a, float b) {
    int diff = a - b;
    return (float)mod(((int)diff + 180), 360) - 180.0;
}
