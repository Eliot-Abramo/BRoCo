"""
setup.py file for SWIG wrapper
"""

from distutils.core import setup, Extension


roco_module = Extension('_RoCo2', sources=['python_wrap.cxx', '../IOBus.cpp',
                                           '../MessageBus.cpp',
                                           '../NetworkBus.cpp',
                                           '../NetworkClientIO.cpp',
                                           '../NetworkServerIO.cpp'])

setup(name='RoCo',
      version='0.1',
      author="Aurelio",
      description="""RoCo Wrapper""",
      ext_modules=[roco_module],
      py_modules=["RoCo"],
      )
