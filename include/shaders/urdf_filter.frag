#version 140
in vec4 normal;
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
  float sensor_depth = texelFetch (depth_texture, int(gl_FragCoord.y)*width + int(gl_FragCoord.x)).x;
  float virtual_depth = to_linear_depth (gl_FragCoord.z);
  float should_filter = float(sensor_depth > (virtual_depth - max_diff));

  // first color attachment: sensor depth image
  gl_FragData[0] = vec4 (sensor_depth, sensor_depth, sensor_depth, 1.0);

  // second color attachment: opengl depth image
  gl_FragData[1] =  mix(vec4(sensor_depth, sensor_depth, sensor_depth, 1.0), vec4(replace_value, 0.0, 0.0, 1.0), should_filter);

  // third color attachment: normal visualization
  gl_FragData[2] = normal * 0.5 + 0.5;

  // fourth color attachment: difference image
  gl_FragData[3] = vec4(should_filter, should_filter, should_filter, 0.0);
}


