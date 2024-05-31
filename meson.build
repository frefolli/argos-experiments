project('prez', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20'])

lua = dependency('lua')
qt5 = dependency('qt5', modules: ['OpenGL'])
argos3 = dependency('argos3')

controllers = library('controllers', [
  './src/controllers/eyebot_controller.cc'
], include_directories: ['./include'], dependencies: [lua, argos3])

#loop_functions = library('loop_functions', [
#  './src/loop_functions/foraging_loop_functions.cc',
#  './src/loop_functions/foraging_qt_user_functions.cc'
#], include_directories: ['./include'], dependencies: [lua, qt5, argos3], link_with: [controllers])
