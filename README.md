# AirSim-Docker


## Set up Unreal Project (https://microsoft.github.io/AirSim/unreal_custenv/)
### In Windows Machine
1. Create new project with custom environment of choice.
1. From AirSim/Unreal/, copy Plugins/ into the Unreal project folder.
1. In Unreal Engine, edit .uproject file so that it looks like this:

{
    "FileVersion": 3,
    "EngineAssociation": "4.24",
    "Category": "Samples",
    "Description": "",
    "Modules": [
        {
            "Name": "LandscapeMountains",
            "Type": "Runtime",
            "LoadingPhase": "Default",
            "AdditionalDependencies": [
                "AirSim"
            ]
        }
    ],
    "TargetPlatforms": [
        "MacNoEditor",
        "WindowsNoEditor"
    ],
    "Plugins": [
        {
            "Name": "AirSim",
            "Enabled": true
        }
    ]
}
