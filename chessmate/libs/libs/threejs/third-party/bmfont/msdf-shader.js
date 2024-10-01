var MSDFShader = 
{
  uniforms		: 	
  {
    map			: 	{ type: 't', value: null },
    color		: 	{ type: 'v3', value: new THREE.Color('#fff') },
	alphaTest	: 	{ type: 'f', value: 0.5 },
    opacity		: 	{ type: 'f', value: 1.0 },
	negate		: 	{ type: 'b', value: true }
  },

  vertexShader: 
  [
    'varying vec2 vUv;',
	'#include <clipping_planes_pars_vertex>',
    'void main(void) {',
	'  #include <begin_vertex>',
    '  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);',
    '  vUv = uv;',
	'  #include <project_vertex>',
	'  #include <clipping_planes_vertex>',
    '}'
  ].join('\n'),

  fragmentShader: 
  [
    '#ifdef GL_OES_standard_derivatives',
    '#extension GL_OES_standard_derivatives: enable',
    '#endif',

	'#include <clipping_planes_pars_fragment>',

    'precision highp float;',
    'uniform float alphaTest;',
    'uniform float opacity;',
    'uniform sampler2D map;',
    'uniform vec3 color;',
	'uniform bool negate;',
    'varying vec2 vUv;',

    'float median(float r, float g, float b) {',
	'  #include <clipping_planes_fragment>',
    '  return max(min(r, g), min(max(r, g), b));',
    '}',

    // FIXME: Experimentally determined constants.
    '#define BIG_ENOUGH 0.001',
    '#define MODIFIED_ALPHATEST (0.02 * isBigEnough / BIG_ENOUGH)',

    'void main() {',
    '  vec3 sample = texture2D(map, vUv).rgb;',
    '  if (negate) { sample = 1.0 - sample; }',

    '  float sigDist = median(sample.r, sample.g, sample.b) - 0.5;',
    '  float alpha = clamp(sigDist / fwidth(sigDist) + 0.5, 0.0, 1.0);',
    '  float dscale = 0.353505;',
    '  vec2 duv = dscale * (dFdx(vUv) + dFdy(vUv));',
    '  float isBigEnough = max(abs(duv.x), abs(duv.y));',

    '  // Do modified alpha test.',
    '  if (alpha < alphaTest * MODIFIED_ALPHATEST) { discard; return; }',
    '  gl_FragColor = vec4(color.xyz, alpha * opacity);',
    '}'
  ].join('\n')

};
