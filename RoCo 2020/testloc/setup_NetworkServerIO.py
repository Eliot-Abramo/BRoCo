from distutils.core import setup, Extension


NetworkServerIO_module = Extension('_NetworkServerIO',
                                   sources=['NetworkServerIO_wrap.cxx',
                                            'NetworkServerIO.cpp'])

setup(name='NetworkServerIO',
      version='0.1',
      author="Aurelio",
      description="""Simple swig example""",
      ext_modules=[NetworkServerIO_module],
      py_modules=["NetworkServerIO"],
      )
