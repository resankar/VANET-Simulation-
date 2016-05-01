# VANET-Simulation-

Both the parts of this project have been coded in Matlab and the implementation of the following model is depicted as Matrix in Matlab for the first part of the project. The position, speed and the acceleration of the vehicles are stored in matrix form in Matlab and to indicate the presence of a vehicle, we insert a “1” in the respective matrix at the required location. This way when the vehicle moves forward the position of “1” is incremented and moves to a new position. Also the position of the vehicle is stored in a vector since the exact position of vehicle is required in cases where the speed is random and varying.

# Instructions to Execute the Code
For executing the first part of the code, the code is designed to run for 100 to 500 vehicles controlled by the variable No_of_cars_in_a_lane, the other looping variables are loop_to_average and time, to run the code for a whole set of 10 min, the variable time is to be set to 60 and the loop average is to execute 5 times to average out the values 5 times.
For the data plots this condition was changed from 1, 10 and 60 to generate the plots.
Please note to get the correct average also the variables limit and increment should be declared (which depends on the looping constraint for time) which is used to calculate the average vehicles.
Part 2 of the implementation can be executed directly as it is, we need to select the points of the roads defined by nodes and the RSU in first case, for the second implementation it is required to select the 4 points for two roads.
