from setuptools import find_packages, setup

setup(
    name="saprog_simulation",
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/saprog_simulation"]),
        ("share/saprog_simulation", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pau Solsona Boada",
    maintainer_email="pausolsonaboada@gmail.com",
    description="On-demand Gazebo simulation",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "simulation_node = saprog_simulation.node:main",
    ]},
)
