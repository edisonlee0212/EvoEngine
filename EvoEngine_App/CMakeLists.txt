add_subdirectory(Demo)
add_subdirectory(EcoSysLab)
add_subdirectory(DigitalAgriculture)
add_subdirectory(DatasetGeneration)

set_property(TARGET DemoApp PROPERTY FOLDER "Apps")
set_property(TARGET EcoSysLabApp PROPERTY FOLDER "Apps")
set_property(TARGET LogGraderApp PROPERTY FOLDER "Apps")
set_property(TARGET DatasetGeneratorApp PROPERTY FOLDER "Apps")

set_property(TARGET DigitalAgricultureApp PROPERTY FOLDER "Apps")