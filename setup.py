from setuptools import setup

setup(
    name='2f85-python-driver',
    version='0.0.1',
    description='A simple independant Python driver to use the Robotiq 2F-85 Gripper.',
    author='Philippe Nadeau',
    author_email='philippe.nadeau@robotics.utias.utoronto.ca',
    license='MIT',
    packages=['Robotiq2F85Driver'],
    install_requires=['minimalmodbus>=2.1'],
    python_requires=">=3.8",
    include_package_data=True
)
