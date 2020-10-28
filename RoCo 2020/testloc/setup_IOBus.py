from distutils.core import setup, Extension


IOBus_module = Extension('_IOBus',
                         sources=['IOBus_wrap.cxx',
                                  'IOBus.cpp'])

setup(name='IOBus',
      version='0.1',
      author="Aurelio",
      description="""Simple swig example""",
      ext_modules=[IOBus_module],
      py_modules=["IOBus"],
      )
