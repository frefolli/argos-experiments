project('prez', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3',
                     'cpp_std=c++20'])

lua = dependency('lua')
qt6 = dependency('qt6', modules: ['OpenGL', 'Widgets', 'OpenGLWidgets'])
#moc = import('qt6')

#processed = moc.preprocess(
#  moc_headers: './include/loop_functions/id_loop_functions/id_qtuser_functions.hh',
#)
argos3 = dependency('argos3')

support = library('support', [
  './src/support/squadrons.cc'
], include_directories: ['./include'], dependencies: [argos3])

controllers = library('controllers', [
  './src/controllers/spiri_controller.cc'
], include_directories: ['./include'], dependencies: [lua, argos3], link_with: [support])

loop_functions = library('loop_functions', [
  './src/loop_functions/loop_function.cc',
], include_directories: ['./include'], dependencies: [lua, argos3], link_with: [controllers, support])

user_functions = library('user_functions', [
  './src/user_functions/user_function.cc'
  #, processed
], include_directories: ['./include'], dependencies: [lua, qt6, argos3])
