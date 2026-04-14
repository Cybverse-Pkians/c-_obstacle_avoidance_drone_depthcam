#include <stdio.h>
#include <math.h>
#define PI 3.14159265

// Global variables
float height = 0, velocity = 0;
float fov[2] = {0, 0};        // fov[0] = HFOV, fov[1] = DFOV
float boundary[2] = {0, 0};   // boundary[0] = length, boundary[1] = breadth
float overlap[2] = {0, 0};    // overlap[0] = frontal, overlap[1] = side
float il = 0, ib = 0;         // image footprint on ground
float total_time = 0;         // total mapping time
int number_of_images = 0;     // final number of images

// Function declarations
void showMenu();
void getInput();
void calculateFOV();
int calculateImages();
void displayResult();

float deg2rad(float angle) {
    return angle * PI / 180.0f;
}

int main() {
    int choice;

    while (1) {
        showMenu();
        printf("Enter your choice: ");
        scanf("%d", &choice);

        if (choice == 1) {
            getInput();
            printf("\nInputs updated successfully.\n");
        }
        else if (choice == 2) {
            calculateFOV();                 // uses global height and fov[]
            number_of_images = calculateImages();  // uses globals, sets total_time
            displayResult();                // prints global number_of_images & total_time
        }
        else if (choice == 3) {
            printf("Exiting application.\n");
            break;
        }
        else {
            printf("Invalid option! Try again.\n");
        }

        printf("\n");
    }

    return 0;
}

void showMenu() {
    printf("===========================\n");
    printf("  Drone Mapping Calculator \n");
    printf("===========================\n");
    printf("1. Enter/Update Mapping Inputs\n");
    printf("2. Calculate Images and Time\n");
    printf("3. Exit\n");
}

void getInput() {
    char dimName[2][10] = {"Length", "Breadth"};

    printf("Enter drone height (m): ");
    scanf("%f", &height);

    printf("Enter drone velocity (m/s): ");
    scanf("%f", &velocity);

    printf("Enter HFOV (degrees): ");
    scanf("%f", &fov[0]);

    printf("Enter DFOV (degrees): ");
    scanf("%f", &fov[1]);

    printf("Enter %s of boundary (m): ", dimName[0]);
    scanf("%f", &boundary[0]);

    printf("Enter %s of boundary (m): ", dimName[1]);
    scanf("%f", &boundary[1]);

    printf("Enter frontal overlap percentage: ");
    scanf("%f", &overlap[0]);
    overlap[0] = overlap[0] / 100.0f;

    printf("Enter side overlap percentage: ");
    scanf("%f", &overlap[1]);
    overlap[1] = overlap[1] / 100.0f;
}

void calculateFOV() {
    float hfov_half_rad = deg2rad(fov[0] / 2.0f);
    float dfov_half_rad = deg2rad(fov[1] / 2.0f);

    il = 2.0f * height * tanf(hfov_half_rad);

    float tan_df = tanf(dfov_half_rad);
    float tan_hf = tanf(hfov_half_rad);
    float sq = tan_df * tan_df - tan_hf * tan_hf;

    if (sq <= 0.0f) {
        ib = il;
    } else {
        ib = 2.0f * height * sqrtf(sq);
    }
}

// Try both ways of using il/ib and pick the minimum images
int calculateImages() {
    float frontal_step1 = il * (1.0f - overlap[0]);
    float side_step1    = ib * (1.0f - overlap[1]);
    if (frontal_step1 <= 0) frontal_step1 = il;
    if (side_step1    <= 0) side_step1    = ib;

    int cols1 = (int)ceil(boundary[0] / frontal_step1);
    int rows1 = (int)ceil(boundary[1] / side_step1);
    if (cols1 < 1) cols1 = 1;
    if (rows1 < 1) rows1 = 1;
    int images1 = cols1 * rows1;

    float time1 = 0.0f;
    if (velocity > 0) {
        time1 = (boundary[0] / velocity) * rows1;
    }

    float frontal_step2 = ib * (1.0f - overlap[0]);
    float side_step2    = il * (1.0f - overlap[1]);
    if (frontal_step2 <= 0) frontal_step2 = ib;
    if (side_step2    <= 0) side_step2    = il;

    int cols2 = (int)ceil(boundary[0] / frontal_step2);
    int rows2 = (int)ceil(boundary[1] / side_step2);
    if (cols2 < 1) cols2 = 1;
    if (rows2 < 1) rows2 = 1;
    int images2 = cols2 * rows2;

    float time2 = 0.0f;
    if (velocity > 0) {
        time2 = (boundary[0] / velocity) * rows2;
    }

    if (images2 < images1) {
        total_time = time2;
        return images2;
    } else {
        total_time = time1;
        return images1;
    }
}

void displayResult() {
    printf("-----------------------------------\n");
    printf("Number of Images Required: %d\n", number_of_images);
    printf("Estimated Time: %.2f seconds\n", total_time);
    printf("-----------------------------------\n");
}