import QtQuick 2.15
import CustomControls 1.0
import QtQuick.Window 2.15
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.5
import "./"
ApplicationWindow {
    width: 1920
    height: 960
    visible: true
    title: qsTr("Car DashBoard")
    color: "#1E1E1E"
    visibility: "FullScreen"
    property int nextSpeed: 60

    function generateRandom(maxLimit = 70){
        let rand = Math.random() * maxLimit;
        rand = Math.floor(rand);
        return rand;
    }

    function speedColor(value){
        if(value < 60 ){
            return "green"
        } else if(value > 60 && value < 150){
            return "yellow"
        }else{
            return "Red"
        }
    }

    function airConditionColor(value){
        if(value < 26 ){
            return "green"
        }else{
            return "yellow"
        }
    }

    function temperature_color(value){
        if(value < 85 ){
            return "green"
        } else if(value >= 85 && value < 105){
            return "yellow"
        }else{
            return "Red"
        }
    }

    Timer {
        interval: 500
        running: true
        repeat: true
        onTriggered:{

            currentTime.text = Qt.formatDateTime(new Date(), "hh:mm")
        }
    }

    Timer{
        repeat: true
        interval: 3000
        running: true
        onTriggered: {
            nextSpeed = generateRandom()
        }
    }

    Shortcut {
        sequence: "Ctrl+Q"
        context: Qt.ApplicationShortcut
        onActivated: Qt.quit()
    }


    Image {
        id: dashboard
        width: parent.width
        height: parent.height
        anchors.centerIn: parent
        source: "qrc:/assets/Dashboard.svg"

        /*
          Top Bar Of Screen
        */

        Image {
            id: topBar
            width: 1100
            source: "qrc:/assets/Vector 70.svg"

            anchors{
                top: parent.top
                topMargin: 20
                horizontalCenter: parent.horizontalCenter
            }

            Image {
                id: engineState
                property bool indicator: false
                width: 52
                height: 52
                anchors{
                    top: parent.top
                    topMargin: 22
                    leftMargin: 180
                    left: parent.left
                }
                source: (canReceiver.engine_status == 1) ? "qrc:/assets/EngineOn.svg" : "qrc:/assets/EngineOff.svg"
                Behavior on indicator { NumberAnimation { duration: 300 }}
                MouseArea{
                    anchors.fill: parent
                    onClicked: {
                        engineState.indicator = !engineState.indicator
                    }
                }
            }

            Label{
                id: currentTime
                text: Qt.formatDateTime(new Date(), "hh:mm")
                font.pixelSize: 28
                font.family: "Inter"
                font.bold: Font.DemiBold
                color: "#FFFFFF"
                anchors.top: parent.top
                anchors.topMargin: 22
                anchors.horizontalCenter: parent.horizontalCenter
            }

            Label{
                id: currentDate
                text: Qt.formatDateTime(new Date(), "dd/MM/yyyy")
                font.pixelSize: 28
                font.family: "Inter"
                font.bold: Font.DemiBold
                color: "#FFFFFF"
                anchors.right: parent.right
                anchors.rightMargin: 160
                anchors.top: parent.top
                anchors.topMargin: 22
            }
        }



        /*
          Speed Label
        */

        Gauge {
            id: speedLabel
            width: 300
            height: 300
            maximumValue: 250

            value: canReceiver.speed       //  Dữ liệu nhận từ C++

            //speedColor: speedColor(value)  // vẫn giữ nếu bạn cần tô màu tùy giá trị

            anchors.top: parent.top
            anchors.topMargin: Math.floor(parent.height * 0.23)
            anchors.horizontalCenter: parent.horizontalCenter

            Component.onCompleted: forceActiveFocus()

            Behavior on value { NumberAnimation { duration: 500 } }
        }


        /*
          Speed Limit Label
        */

        

        Rectangle{
            id:speedLimit
            width: 100
            height: 100
            radius: height/2
            color: "#f2f2f2"
            border.color: "#FF0000"
            border.width: 10

            anchors.horizontalCenter: parent.horizontalCenter
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 50

            Label{
                id:maxSpeedlabel
                text: canReceiver.speed_limit.toFixed(0)
                font.pixelSize: 35
                font.family: "Inter"
                font.bold: Font.Bold
                color: "#01E6DE"
                anchors.centerIn: parent

                function getRandomInt(min, max) {
                    return Math.floor(Math.random() * (max - min + 1)) + min;
                }
            }
        }

        Image {
            id: speed_Limit_Add_Button
            property bool indicator: false
            width: 40
            height: 40
            source: "qrc:/assets/Add.svg"
            fillMode: Image.PreserveAspectFit
            anchors{
                right: speedLimit.left
                rightMargin: 20
                verticalCenter: speedLimit.verticalCenter
            }
            Behavior on indicator { NumberAnimation { duration: 300 }}
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    canReceiver.speedLimitAdd()
                    canReceiver.UpdateData()
                }
            }
        }

        Image {
            id: speed_Limit_Substract_Button
            property bool indicator: false
            width: 40
            height: 40
            source: "qrc:/assets/Substract.svg"
            fillMode: Image.PreserveAspectFit
            anchors{
                left: speedLimit.right
                leftMargin: 20
                verticalCenter: speedLimit.verticalCenter
            }
            Behavior on indicator { NumberAnimation { duration: 300 }}
            MouseArea {
                anchors.fill: parent
                onClicked: {
                    canReceiver.speedLimitSubtract()
                    canReceiver.UpdateData()
                }
            }
        }

        Image {
            id:car
            width: 90
            height: 90
            anchors{
                bottom: speedLimit.top
                bottomMargin: 30
                horizontalCenter:speedLimit.horizontalCenter
            }
            source: "qrc:/assets/Car.svg"
        }

        // IMGonline.com.ua  ==> Compress Image With


        /*
          Left Road
        */

        Image {
            id: leftRoad
            width: 100
            height: 350
            anchors{
                left: speedLimit.left
                leftMargin: 90
                bottom: parent.bottom
                bottomMargin: 20.50
            }

            source: "qrc:/assets/Vector 2.svg"
        }

        RowLayout{
            spacing: 20

            anchors{
                left: parent.left
                leftMargin: 120
                bottom: parent.bottom
                bottomMargin: 70
            }

            RowLayout{
                spacing: 3

                Image {
                    id: air_Condition_Add_Button
                    property bool indicator: false
                    width: 15             // Đặt lại kích thước mong muốn
                    height: 15
                    source: "qrc:/assets/Add.svg"
                    //fillMode: Image.PreserveAspectFit
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft  // Căn chỉnh trong RowLayout
                    Behavior on indicator { NumberAnimation { duration: 300 }}
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            canReceiver.airConditionTemperatureAdd()
                            canReceiver.UpdateData()
                        }
                    }
                }

                Label{
                    text: canReceiver.air_condition_temperature.toFixed(0)
                    font.pixelSize: 35
                    font.family: "Inter"
                    font.bold: Font.Normal
                    font.capitalization: Font.AllUppercase
                    color: "#FFFFFF"
                    Layout.preferredWidth: 30  // Cố định width cho 3 chữ số
                    horizontalAlignment: Text.AlignRight
                }

                Label{
                    text: "°C"
                    font.pixelSize: 35
                    font.family: "Inter"
                    font.bold: Font.Normal
                    font.capitalization: Font.AllUppercase
                    //opacity: 0.2
                    color: "#ffffffff"
                }
            }

            RowLayout{
                spacing: 1
                Layout.topMargin: 0
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 16 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 19 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 22 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 25 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 28 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 31 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
                Rectangle{
                    width: 20
                    height: 15
                    color: canReceiver.air_condition_temperature.toFixed(0) > 34 ? airConditionColor(canReceiver.air_condition_temperature) : "#a6a6a6"
                }
            }

            RowLayout{
                spacing: 3

                Image {
                    id: air_Condition_Sub_Button
                    property bool indicator: false
                    width: 40                // Đặt lại kích thước mong muốn
                    height: 40
                    source: "qrc:/assets/Substract.svg"
                    fillMode: Image.PreserveAspectFit
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft  // Căn chỉnh trong RowLayout
                    Behavior on indicator { NumberAnimation { duration: 300 }}
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            canReceiver.airConditionTemperatureSubtract()
                            canReceiver.UpdateData()
                        }
                    }
                }
            }

           
        }

        /*
          Right Road
        */

        Image {
            id: rightRoad
            width: 100
            height: 350
            anchors{
                right: speedLimit.right
                rightMargin: 90
                bottom: parent.bottom
                bottomMargin: 20.50
            }

            source: "qrc:/assets/Vector 1.svg"
        }

        /*
          Right Side gear
        */

        RowLayout{
            spacing: 20
            anchors{
                right: parent.right
                rightMargin: 140
                bottom: parent.bottom
                bottomMargin: 80
            }

            Label{
                text: "Ready"
                font.pixelSize: 32
                font.family: "Inter"
                font.bold: Font.Normal
                font.capitalization: Font.AllUppercase
                color: "#32D74B"
            }
            
            Label{
                text: "P"
                font.pixelSize: 32
                font.family: "Inter"
                font.bold: Font.Normal
                font.capitalization: Font.AllUppercase
                color: (canReceiver.transmission_gear == 0)  ?  "#01EBD4"  : "#a6a6a6"
            }

            Label{
                text: "R"
                font.pixelSize: 32
                font.family: "Inter"
                font.bold: Font.Normal
                font.capitalization: Font.AllUppercase
                color: (canReceiver.transmission_gear == 1)  ?  "#01EBD4"  : "#a6a6a6"
            }
            Label{
                text: "N"
                font.pixelSize: 32
                font.family: "Inter"
                font.bold: Font.Normal
                font.capitalization: Font.AllUppercase
                color: (canReceiver.transmission_gear == 2)  ?  "#01EBD4"  : "#a6a6a6"
            }
            Label{
                text: "D"
                font.pixelSize: 32
                font.family: "Inter"
                font.bold: Font.Normal
                font.capitalization: Font.AllUppercase
                color: (canReceiver.transmission_gear == 3)  ?  "#01EBD4"  : "#a6a6a6"
            }
        }

        /*Left Side Icons*/

        Image {
            id:highBeamLightIndicator
            property bool lightOn: true
            width: 60
            height: 70.2
            anchors{
                left: parent.left
                leftMargin: 85
                bottom: secondLeftIndicator.top
                bottomMargin: 30
            }
            source: (canReceiver.light_status ==1) ? "qrc:/assets/HighBeamLightOn.svg" : "qrc:/assets/HighBeamLightOff.svg"
            //Behavior on lightOn { NumberAnimation { duration: 300 }}
            MouseArea{
                anchors.fill: parent
                onClicked: {
                    highBeamLightIndicator.lightOn = !highBeamLightIndicator.lightOn
                }
            }
        }

        Image {
            id:secondLeftIndicator
            property bool headLightOn: true
            width: 51
            height: 51
            anchors{
                left: parent.left
                leftMargin: 85
                bottom: firstLeftIndicator.top
                bottomMargin: 30
            }
            Behavior on headLightOn { NumberAnimation { duration: 300 }}
            source:(canReceiver.light_status ==2) ?  "qrc:/assets/LowBeamLightOn.svg" : "qrc:/assets/LowBeamLightOff.svg"

            MouseArea{
                anchors.fill: parent
                onClicked: {
                    secondLeftIndicator.headLightOn = !secondLeftIndicator.headLightOn
                }
            }
        }

        Image {
            id:firstLeftIndicator
            property bool rareLightOn: false
            width: 51
            height: 51
            anchors{
                left: parent.left
                leftMargin: 85
                bottom: parent.bottom
                bottomMargin: 250
            }
            source: (canReceiver.light_status ==3) ? "qrc:/assets/FogLightOn.svg" : "qrc:/assets/FogLightOff.svg"
            Behavior on rareLightOn { NumberAnimation { duration: 300 }}
            MouseArea{
                anchors.fill: parent
                onClicked: {
                    firstLeftIndicator.rareLightOn = !firstLeftIndicator.rareLightOn
                }
            }
        }

        /*Right Side Icons*/


        Image {
            id:thirdRightIndicator
            property bool indicator: true
            width: 56.83
            height: 36.17
            anchors{
                right: parent.right
                rightMargin: 80
                bottom: secondRightIndicator.top
                bottomMargin: 50
            }
            source: (canReceiver.tire_pressure ==1) ? "qrc:/assets/TirePressOK.svg" : "qrc:/assets/TirePressNotOK.svg"
            Behavior on indicator { NumberAnimation { duration: 300 }}
            MouseArea{
                anchors.fill: parent
                onClicked: {
                    thirdRightIndicator.indicator = !thirdRightIndicator.indicator
                }
            }
        }

        Image {
            id:secondRightIndicator
            property bool indicator: true
            width: 56.83
            height: 36.17
            anchors{
                right: parent.right
                rightMargin: 80
                bottom: firstRightIndicator.top
                bottomMargin: 50
            }
            source: (canReceiver.door_status ==1) ? "qrc:/assets/DoorClosed.svg" : "qrc:/assets/DoorOpen.svg"
            Behavior on indicator { NumberAnimation { duration: 300 }}
            MouseArea{
                anchors.fill: parent
                onClicked: {
                    secondRightIndicator.indicator = !secondRightIndicator.indicator
                }
            }
        }

        Image {
            id:firstRightIndicator
            property bool sheetBelt: true
            width: 36
            height: 45
            anchors{
                right: parent.right
                rightMargin: 85
                bottom: parent.bottom
                bottomMargin: 250
            }
            source: (canReceiver.seat_belt_status ==1) ? "qrc:/assets/SeatBeltOn.svg" : "qrc:/assets/SeatBeltOff.svg"
            Behavior on sheetBelt { NumberAnimation { duration: 300 }}
            MouseArea{
                anchors.fill: parent
                onClicked: {
                    firstRightIndicator.sheetBelt = !firstRightIndicator.sheetBelt
                }
            }
        }

        // Progress Bar
        RadialBar {
            id: radialBar
            anchors{
                verticalCenter: parent.verticalCenter
                left: parent.left
                leftMargin: parent.width / 6
            }

            width: 250
            height: 250
            penStyle: Qt.RoundCap
            dialType: RadialBar.NoDial
            progressColor: "#01E4E0"
            backgroundColor: "#363636ff"  // Màu nền mờ cho phần còn lại của vòng tròn
            dialWidth: 17
            startAngle: 270
            spanAngle: 3.6 * value
            minValue: 0
            maxValue: 100
            value: canReceiver.battery_level
            textFont {
                family: "inter"
                italic: false
                bold: Font.Medium
                pixelSize: 40
            }
            showText: false
            suffixText: ""
            textColor: "#FFFFFF"

            property bool accelerating
            Behavior on value { NumberAnimation { duration: 1000 }}

            ColumnLayout{
                anchors.centerIn: parent
                Label{
                    text: radialBar.value.toFixed(0) + "%"
                    font.pixelSize: 40
                    font.family: "Inter"
                    font.bold: Font.Normal
                    color: "#FFFFFF"
                    Layout.alignment: Qt.AlignHCenter
                }

                Label{
                    text: "Battery level"
                    font.pixelSize: 24
                    font.family: "Inter"
                    font.bold: Font.Normal
                    opacity: 0.8
                    color: "#FFFFFF"
                    Layout.alignment: Qt.AlignHCenter
                }
            }
        }

        ColumnLayout{
            spacing: 40

            anchors{
                top: parent.top
                topMargin: 230
                right: parent.right
                rightMargin: 180
            }

            RowLayout{
                spacing: 20
                Image {
                    width: 20
                    height:20
                    source: "qrc:/assets/road.svg"
                }

                ColumnLayout{
                    Label{
                        text: canReceiver.arrived_distance.toFixed(0) + "KM"
                        font.pixelSize: 25
                        font.family: "Inter"
                        font.bold: Font.Normal
                        opacity: 0.8
                        color: "#FFFFFF"
                    }
                    Label{
                        text: "Drived Distance"
                        font.pixelSize: 18
                        font.family: "Inter"
                        font.bold: Font.Normal
                        opacity: 0.8
                        color: "#FFFFFF"
                    }
                }
            }
            RowLayout{
                spacing: 20
                Image {
                    width: 20
                    height: 20
                    source: "qrc:/assets/fuel.svg"
                }

                ColumnLayout{
                    Label{
                        text: canReceiver.remain_distance.toFixed(0) + "KM"
                        font.pixelSize: 25
                        font.family: "Inter"
                        font.bold: Font.Normal
                        opacity: 0.8
                        color: "#FFFFFF"
                    }
                    Label{
                        text: "Remain Distance"
                        font.pixelSize: 18
                        font.family: "Inter"
                        font.bold: Font.Normal
                        opacity: 0.8
                        color: "#FFFFFF"
                    }
                }
            }
            RowLayout{
                spacing: 20
                Image {
                    width: 20
                    height: 20
                    source: "qrc:/assets/speedometer.svg"
                }

                ColumnLayout{
                    Label{
                        text: canReceiver.drived_time.toFixed(0) + " minutes"
                        font.pixelSize: 25
                        font.family: "Inter"
                        font.bold: Font.Normal
                        opacity: 0.8
                        color: "#FFFFFF"
                    }
                    Label{
                        text: "Drived Time"
                        font.pixelSize: 18
                        font.family: "Inter"
                        font.bold: Font.Normal
                        opacity: 0.8
                        color: "#FFFFFF"
                    }
                }
            }
        }
    }
}