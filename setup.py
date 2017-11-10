from setuptools import setup

setup(name='pyopto',
      py_modules = ['pyopto'],
      version='0.1',
      description='Python interface for Optotune vari-focal lens.',
      url='',
      author='Ha0k',
      author_email='ha0k@haok.lol',
      keywords = "optotune lens driver",
      license='MIT',
      install_requires=['pyserial'],
      zip_safe=False)
