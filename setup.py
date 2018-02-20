from distutils.core import setup
from distutils.extension import Extension
from Cython.Build import cythonize
import os

# dirty fix (gcc compiler -> g++ linker incompatibility)
os.environ["CC"] = "g++"
c_incdirs = ['src/cpp/includes/']
c_args = ['-std=c++11']

setup(
    name='amcts',
    version='alpha0.1',
    description='Asynchronous Monte-Carlo Tree Search (AMCTS) implementation \
for learning to branch in CPLEX.',
    author='Maxime Gasse',
    author_email='maxime.gasse@gmail.com',
    url='https://github.com/ds4dm/amcts-cplex',
    license='GNU GPL v3',
    requires=[
        'cplex (>=12.8)',
        'numpy'],
    package_dir={'': 'src'},
    ext_modules=cythonize(
        module_list=[
            Extension(
                name='amcts.comm',
                sources=[
                    'src/amcts/comm.pyx',
                    'src/cpp/pipe.c'],
                include_dirs=c_incdirs,
                extra_compile_args=c_args),
            Extension(
                name='amcts.worker',
                sources=[
                    'src/amcts/worker.pyx',
                    'src/cpp/pipe.c'],
                include_dirs=c_incdirs,
                extra_compile_args=c_args),
            Extension(
                name='amcts.__init__',
                sources=[
                    'src/amcts/__init__.pyx',
                    'src/cpp/pipe.c',
                    'src/cpp/amcts.cpp'],
                include_dirs=c_incdirs,
                extra_compile_args=c_args),
        ],
        compiler_directives={
            'boundscheck': False,
            'wraparound': False,
            'cdivision': True,
        }),
)
