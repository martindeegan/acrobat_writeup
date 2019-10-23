R = [  0.9900333,  0.0099667,  0.1404804;
   0.0099667,  0.9900333, -0.1404804;
  -0.1404804,  0.1404804,  0.9800666 ];

position_origin = [0; 0; 0];
position_camera = [4; 0; 1];
position_imu    = [0; 0; -3];

conversion_matrix = [0 0 -1; 
                     1 0  0; 
                     0 -1 0];

conversion_matrix * position_origin
conversion_matrix * position_camera
conversion_matrix * position_imu
