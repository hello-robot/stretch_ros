from setuptools import setup

package_name = 'hello_helpers'

setup(
    name=package_name,
    version='0.2.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    url='',
    license='Apache License 2.0',
    author='Hello Robot Inc.',
    author_email='support@hello-robot.com',
    description='The hello_helpers package',
)
