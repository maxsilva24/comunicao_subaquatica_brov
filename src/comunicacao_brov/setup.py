from setuptools import setup

package_name = 'comunicacao_brov'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='profm',
    maintainer_email='maxwellfdasilva@gmail.com',
    description='Código para a Comunicação para o BROV',
    license='Liberado',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'no_comunicacao_brov = comunicacao_brov.no_comunicacao_brov:main'
        ],
    },
)
