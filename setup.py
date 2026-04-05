from setuptools import setup, find_packages

setup(
    name="dex-control",
    version="0.1.0",
    description="Remote control framework for Franka arm using gRPC",
    author="Jinzhou Li",
    author_email="your.email@example.com",
    packages=find_packages(),
    install_requires=[
        "grpcio",
        "protobuf",
        "numpy",
    ],
    python_requires=">=3.10.0",
    entry_points={
        # No CLI scripts yet, can add later if desired
    }
)
