*** Settings ***
Documentation    Test Zenoh session with talker node
Library          ros2_client.ROS2ClientLibrary

*** Test Cases *** 
Test Zenoh Session
    [Documentation]    Test Zenoh Session
    [Tags]    zenoh    session    talker
    [Teardown]    Destroy Zenoh Session
    Create Zenoh Session

    Run Node     demo_nodes_cpp    talker
    Sleep    3s

    ${nodes}=    List Nodes
    Should Contain    ${nodes}    /talker
    Create Subscriber    /chatter    std_msgs/msg/String
    ${messages}=   Wait For Message    /chatter    timeout=10
    Should Contain    ${messages}[data]    Hello World

    Destroy Zenoh Session

Test List Topics
    [Documentation]    Test List Topics
    [Tags]    zenoh    list    topics
    [Teardown]    Destroy Zenoh Session
    Create Zenoh Session

    ${process}=    Run Node     demo_nodes_cpp    talker
    Sleep    3s

    ${nodes}=    List Nodes
    Should Contain    ${nodes}    /talker
    Create Subscriber    /chatter    std_msgs/msg/String
    ${messages}=   Wait For Message    /chatter    timeout=10
    Should Contain    ${messages}[data]    Hello World

    ${topics}=    List Topics
    Should Contain    ${topics}    /chatter

*** Keywords ***
Create Zenoh Session
    [Documentation]    Create a Zenoh session
    [Tags]    zenoh    session
    ${process}=    Run Node     rmw_zenoh_cpp     rmw_zenohd
    Log    Zenoh session started with PID: ${process}


Destroy Zenoh Session
    [Documentation]    Destroy a Zenoh session
    [Tags]    zenoh    session
    Kill Process By Name    rmw_zenohd

    Kill Process By Name    talker
