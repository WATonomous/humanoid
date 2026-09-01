## Documenting:
Documentation should be used as a blueprint for development, then filled in when specifics are established. 

The documentation of this repository follows a package level scheme. As shown below:
```
common_msgs/
├── README.md ──────────────> overview (package)
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── ArmPose.msg
    └── ...
```

### Component Level 
- Overview of component as a whole

### Package Level   
- **Purpose** – What the package does and its role within the component.  
- **Inputs & Outputs** – Data flow, including message types, service calls, or file interactions.  
- **Key Features** – Key classes, nodes, or scripts, along with their relationships.  
- **Usage** – How to build, run, and test the package.  
- **Configuration** – Relevant parameters, environment variables, or dependencies.

### System Architecture 
The system architecture can be viewed in this [document](Architecture_Map.pdf), along with the .odg file (use libre draw to edit).

### Infrastructure Documentation
- [Messages](../src/common_msgs/README.md)
