# Kidnapped Vehicle Project

The purpose of the project is to implement a particle filter able to localize a vehicle position and yaw to within the values specified in a set of given parameters: `max_translation_error` and `max_yaw_error`.


### Project implementation steps

1. Initialization
- A number of 100 particles has been chosen
- Initialize all particles to the first position and weight 1 
- Add random Gaussian noise to each particle

2. Prediction
- Add measurements to each particle 
- Add random noise to each particle using a normal (Gaussian) distribution

3. Update weights
- Filter out any landmarks that are out of the sensor range for the current particle
- Transform observations from vehicle coordinates to map coordinates
- Associate the predicted measurement that is closest to each observed measurement
- Calculate weights using a multi-variate Gaussian distribution

4. Resample particles



### Running the Code

Once the implementation is correct, running the project with the Term 2 simulator produces the following outputs:

| Simulator output | Console output |
| --- | --- |
| ![Your particle filter passed](https://raw.githubusercontent.com/sorix6/CarND-Kidnapped-Vehicle-Project/master/images/filter_pass.JPG) | ![Console output](https://raw.githubusercontent.com/sorix6/CarND-Kidnapped-Vehicle-Project/master/images/console.JPG) |
