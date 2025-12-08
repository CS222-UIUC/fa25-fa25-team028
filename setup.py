from setuptools import setup, find_packages

setup(
    name="engine",
    version="0.1",
    description="A simple 2D physics engine",
    long_description=open("README.md", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    author="Team 28",
    python_requires=">=3.9",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=[
        "numpy>=1.24",
    ],
    extras_require={
        "dev": ["pytest>=7", "pygame>=2; platform_system!='Darwin'"], 
    },
    include_package_data=True, 
    entry_points={
        "console_scripts": [
            "single-spring-demo=engine.demos.single_spring_demo:main",
            "pygame-spring-demo=engine.demos.pygame_spring_demo:main",
            "rigidbody2d-demo=engine.demos.rigidbody2d_pygame_demo:main",
            "rigidbody2d-sandbox=engine.demos.rigidbody2d_sandbox:main",
            "physics-sandbox-showcase=engine.demos.physics_sandbox_showcase:main",
        ]
    },
)