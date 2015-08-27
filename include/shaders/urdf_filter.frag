#version 140
varying vec3 normal;
uniform int width;
uniform int height;
uniform samplerBuffer depth_texture;

uniform float replace_value;

uniform float z_near;
uniform float z_far;

uniform float max_diff;

float to_linear_depth (float d)
{
  return (z_near * z_far / (z_near - z_far)) / (d - z_far / (z_far - z_near));
}

void main(void)
{
  // first color attachment: sensor depth image
  float sensor_depth = texelFetch (depth_texture, int(gl_FragCoord.y)*width + int(gl_FragCoord.x)).x;
  sensor_depth = sensor_depth/1000.0;
  gl_FragData[0] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);
  
  
  // third color attachment: normal visualization
  gl_FragData[2] = vec4 ((normal.x + 1.0) * 0.5,
                         (normal.y + 1.0) * 0.5,
                         (normal.z + 1.0) * 0.5,
                         1.0);
  
  

  // second color attachment: opengl depth image
  float virtual_depth = to_linear_depth (gl_FragCoord.z);
//   if(virtual_depth>4.0)
//     virtual_depth = 0.0;
    
  bool keep_point = sensor_depth < virtual_depth;
  if(keep_point)
    gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);
  else
    gl_FragData[1] = vec4 (0.0, 0.0, 0.0, 1.0);
    
  // full robot image
//   gl_FragData[1] = vec4 (virtual_depth, virtual_depth, virtual_depth, 1.0);
  
  // depth image
//   gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);

  // depth_image + piece of robot to robot
//   bool bool_filter = (virtual_depth - sensor_depth) > max_diff;
//   if (bool_filter)
//     gl_FragData[1] = vec4 (255.0, 255.0, 255.0, 1.0);
//   else
//     gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);
  
  
//   gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);



  // fourth color attachment: difference image
//   bool should_filter = (virtual_depth - sensor_depth) > max_diff;


//   if (should_filter)
//     gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0); //  that should make it red
//   else
//     gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0); // grayscale depth image
    
//     rajoute le robot
  /*if (!should_filter)
    gl_FragData[1] = vec4 (replace_value, 0.0, 0.0, 1.0); //  that should make it red
  else
    gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0); // grayscale depth image
  */  






/*


///////////INITIAL CODE /////////////////

  // first color attachment: sensor depth image
  float sensor_depth = texelFetch (depth_texture, int(gl_FragCoord.y)*width + int(gl_FragCoord.x)).x;
  sensor_depth = sensor_depth/1000;
  gl_FragData[0] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);

  // second color attachment: opengl depth image
  float virtual_depth = to_linear_depth (gl_FragCoord.z);
  gl_FragData[1] = vec4 (virtual_depth, virtual_depth, virtual_depth, 1.0);

  // third color attachment: normal visualization
  gl_FragData[2] = vec4 ((normal.x + 1.0) * 0.5,
                         (normal.y + 1.0) * 0.5,
                         (normal.z + 1.0) * 0.5,
                         1.0);

  // fourth color attachment: difference image
  bool should_filter = (virtual_depth - sensor_depth) > max_diff;

  if (should_filter)
    gl_FragData[1] = vec4 (replace_value, 0.0, 0.0, 1.0); //  that should make it red
  else
    gl_FragData[1] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0); // grayscale depth image
*/



}

		
