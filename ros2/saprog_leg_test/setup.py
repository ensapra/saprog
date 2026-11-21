from setuptools import find_packages, setup

setup(
    name="saprog_leg_test",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/saprog_leg_test"]),
        ("share/saprog_leg_test", ["package.xml"]),
    ],
    install_requires=["setuptools", "mujoco>=3.2,<4"],
    zip_safe=True,
    maintainer="Pau Solsona Boada",
    maintainer_email="pausolsonaboada@gmail.com",
    description="Isolated physical crank-linkage test rig",
    license="Apache-2.0",
    entry_points={"console_scripts": ["leg_test_node = saprog_leg_test.node:main"]},
)
