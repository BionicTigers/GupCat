//package org.firstinspires.ftc.teamcode.drivers
//
//import com.qualcomm.robotcore.hardware.HardwareDevice
//import com.qualcomm.robotcore.hardware.I2cAddr
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch
//import com.qualcomm.robotcore.hardware.I2cDeviceSynch.ReadWindow
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice
//import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
//import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
//import org.firstinspires.ftc.teamcode.utils.Vector3
//import java.nio.ByteBuffer
//
//enum class Register(val bVal: Int) {
//    FIRST(0),
//
//    MAG_RADIUS_MSB(0x6A),
//    MAG_RADIUS_LSB(0x69),
//    ACC_RADIUS_MSB(0x68),
//    ACC_RADIUS_LSB(0x67),
//    GYR_OFFSET_Z_MSB(0x66),
//    GYR_OFFSET_Z_LSB(0x65),
//    GYR_OFFSET_Y_MSB(0x64),
//    GYR_OFFSET_Y_LSB(0x63),
//    GYR_OFFSET_X_MSB(0x62),
//    GYR_OFFSET_X_LSB(0x61),
//    MAG_OFFSET_Z_MSB(0x60),
//    MAG_OFFSET_Z_LSB(0x5F),
//    MAG_OFFSET_Y_MSB(0x5E),
//    MAG_OFFSET_Y_LSB(0x5D),
//    MAG_OFFSET_X_MSB(0x5C),
//    MAG_OFFSET_X_LSB(0x5B),
//    ACC_OFFSET_Z_MSB(0x5A),
//    ACC_OFFSET_Z_LSB(0x59),
//    ACC_OFFSET_Y_MSB(0x58),
//    ACC_OFFSET_Y_LSB(0x57),
//    ACC_OFFSET_X_MSB(0x56),
//    ACC_OFFSET_X_LSB(0x55),
//
//    AXIS_MAP_SIGN(0x41),
//    AXIS_MAP_CONFIG(0x41),
//
//    OPR_MODE(0x3D),
//
//    SYS_ERROR(0x3A),
//    SYS_STATUS(0x39),
////    INT_STATUS(0x37),
////    ST_RESULT(0x36),
//    CALIB_STAT(0x35),
//
//    GRV_DATA_Z_MSB(0x33),
//    GRV_DATA_Z_LSB(0x32),
//    GRV_DATA_Y_MSB(0x31),
//    GRV_DATA_Y_LSB(0x30),
//    GRV_DATA_X_MSB(0x2F),
//    GRV_DATA_X_LSB(0x2E),
//    LIA_DATA_Z_MSB(0x2D),
//    LIA_DATA_Z_LSB(0x2C),
//    LIA_DATA_Y_MSB(0x2B),
//    LIA_DATA_Y_LSB(0x2A),
//    LIA_DATA_X_MSB(0x29),
//    LIA_DATA_X_LSB(0x28),
//    QUAT_DATA_Z_MSB(0x27),
//    QUAT_DATA_Z_LSB(0x26),
//    QUAT_DATA_Y_MSB(0x25),
//    QUAT_DATA_Y_LSB(0x24),
//    QUAT_DATA_X_MSB(0x23),
//    QUAT_DATA_X_LSB(0x22),
//    QUAT_DATA_W_MSB(0x21),
//    QUAT_DATA_W_LSB(0x20),
//    EUL_PITCH_MSB(0x1F),
//    EUL_PITCH_LSB(0x1E),
//    EUL_ROLL_MSB(0x1D),
//    EUL_ROLL_LSB(0x1C),
//    EUL_HEADING_MSB(0x1B),
//    EUL_HEADING_LSB(0x1A),
//    GYR_DATA_Z_MSB(0x19),
//    GYR_DATA_Z_LSB(0x18),
//    GYR_DATA_Y_MSB(0x17),
//    GYR_DATA_Y_LSB(0x16),
//    GYR_DATA_X_MSB(0x15),
//    GYR_DATA_X_LSB(0x14),
//    MAG_DATA_Z_MSB(0x13),
//    MAG_DATA_Z_LSB(0x12),
//    MAG_DATA_Y_MSB(0x11),
//    MAG_DATA_Y_LSB(0x10),
//    MAG_DATA_X_MSB(0x0F),
//    MAG_DATA_X_LSB(0x0E),
//    ACC_DATA_Z_MSB(0x0D),
//    ACC_DATA_Z_LSB(0x0C),
//    ACC_DATA_Y_MSB(0x0B),
//    ACC_DATA_Y_LSB(0x0A),
//    ACC_DATA_X_MSB(0x09),
//    ACC_DATA_X_LSB(0x08),
//
////    PAGE_ID(0x07),
////    BL_REV_ID(0x06),
////    SW_REV_ID_LSB(0x05),
////    SW_REV_ID_MSB(0x04),
//
//    GYR_ID(0x03),
//    MAG_ID(0x02),
//    ACC_ID(0x01),
//    CHIP_ID(0x00),
//
//    LAST(CHIP_ID.bVal)
//}
//
//@I2cDeviceType
//@DeviceProperties(xmlTag = "DFRobot10DOFIMU", name = "DFRobot 10 DOF Gravity IMU")
//class IMU(
//    deviceClient: I2cDeviceSynch?,
//    deviceClientIsOwned: Boolean ,
//) : I2cDeviceSynchDevice<I2cDeviceSynch>(deviceClient, deviceClientIsOwned) {
//    // pages 24-5 in the datasheet describe how to do the axis mapping
//    val axisVals = 0b00100100 // 0b00 100100 is the default value
//    val axisSign = 0b00000000 // 0b00000 000 is the default value
//
//    private val calibration: ShortArray = shortArrayOf(
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//        0x00,
//    )
//
//    init {
//        this.deviceClient.i2cAddress = I2cAddr.create8bit(0x28)
//        this.setOptimalReadWindow()
//        super.registerArmingStateCallback(false) // Deals with USB cables getting unplugged
//        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
//        deviceClient!!.engage()
//
//        setAxisMapConfig(axisVals.toShort())
//        setAxisMapSign(axisSign.toShort())
//
//        writeCalibrationValues(calibration)
//    }
//
//    override fun getManufacturer(): HardwareDevice.Manufacturer {
//        return HardwareDevice.Manufacturer.DFRobot
//    }
//    override fun getDeviceName(): String {
//        return "DF Robot 10 DOF Gravity IMU"
//    }
//    override fun doInitialize(): Boolean {
//        return true
//    }
//
//    protected fun setOptimalReadWindow() {
//        val readWindow = ReadWindow(
//            Register.FIRST.bVal,
//            Register.LAST.bVal - Register.FIRST.bVal + 1,
//            I2cDeviceSynch.ReadMode.REPEAT
//        )
//        deviceClient.readWindow = readWindow
//    }
//
//    protected fun writeShort(register: Register, value: Short) {
//        deviceClient.write(register.bVal, ByteBuffer.allocate(2).putShort(value).array())
//    }
//    protected fun readShort(register: Register): Short {
//        return ByteBuffer.wrap(deviceClient.read(register.bVal, 2)).short
//    }
//
//    private fun combineMSBandLSB(msb: Short, lsb: Short): Short {
//        return ((msb.toInt() shl 8) or (lsb.toInt() and 0xFF)).toShort()
//    }
//    private fun separateMSBandLSB(value: Short): Pair<Short, Short> {
//        return Pair((value.toInt() shr 8).toShort(), (value.toInt() and 0xFF).toShort())
//    }
//
//    // only the first 6 bits will be used, the rest will be ignored
//    private fun setAxisMapConfig(axisMapConfig: Short) {
//        writeShort(Register.AXIS_MAP_CONFIG,
//            (readShort(Register.AXIS_MAP_CONFIG).toInt() and 0xC0 or
//                    axisMapConfig.toInt() and 0x3F).toShort())
//    }
//    // only the first 3 bits will be used, the rest will be ignored
//    private fun setAxisMapSign(axisMapSign: Short) {
//        writeShort(Register.AXIS_MAP_SIGN,
//            (readShort(Register.AXIS_MAP_SIGN).toInt() and 0xF8 or
//                    axisMapSign.toInt() and 0x07).toShort())
//    }
//
//    // pages 21-3 in the datasheet describe operation modes
//    // only the first 4 bits will be used, the rest will be ignored
//    fun setOperationMode(oprMode: Short) {
//        writeShort(Register.OPR_MODE,
//            (readShort(Register.AXIS_MAP_SIGN).toInt() and 0xF0 or
//                oprMode.toInt() and 0x0F).toShort())
//    }
//    fun setOperationModeNDOF() {
//        setOperationMode(0b1100)
//    }
//    fun setOperationModeConfig() {
//        setOperationMode(0b0000)
//    }
//
//    fun getChipID(): Short {
//        return readShort(Register.CHIP_ID)
//    }
//    fun getAccID(): Short {
//        return readShort(Register.ACC_ID)
//    }
//    fun getMagID(): Short {
//        return readShort(Register.MAG_ID)
//    }
//    fun getGyrID(): Short {
//        return readShort(Register.GYR_ID)
//    }
//    fun getSysError(): Short {
//        return readShort(Register.SYS_ERROR)
//    }
//    fun getSysStatus(): Short {
//        return readShort(Register.SYS_STATUS)
//    }
//    fun getCalibrationStatus(): Short {
//        return readShort(Register.CALIB_STAT)
//    }
//
//    fun getGravityVector(): Vector3 {
//        return Vector3(
//            getGravDataX(),
//            getGravDataY(),
//            getGravDataZ()
//        )
//    }
//    fun getLinearAcceleration(): Vector3 {
//        return Vector3(
//            getLinearAccDataX(),
//            getLinearAccDataY(),
//            getLinearAccDataZ()
//        )
//    }
//    fun getEulerAngle(): Vector3 {
//        return Vector3(
//            getEulDataPitch(),
//            getEulDataRoll(),
//            getEulDataHeading()
//        )
//    }
//    fun getAngularVelocity(): Vector3 {
//        return Vector3(
//            getGyrDataX(),
//            getGyrDataY(),
//            getGyrDataZ()
//        )
//    }
//    fun getMagneticFieldStrength(): Vector3 {
//        return Vector3(
//            getMagDataX(),
//            getMagDataY(),
//            getMagDataZ()
//        )
//    }
//    fun getAcceleration(): Vector3 {
//        return Vector3(
//            getAccDataX(),
//            getAccDataY(),
//            getAccDataZ()
//        )
//    }
//
//    fun getGravDataZ(): Short {
//        return combineMSBandLSB(
//            readShort(Register.GRV_DATA_Z_MSB),
//            readShort(Register.GRV_DATA_Z_LSB))
//    }
//    fun getGravDataY(): Short {
//        return combineMSBandLSB(
//            readShort(Register.GRV_DATA_Y_MSB),
//            readShort(Register.GRV_DATA_Y_LSB))
//    }
//    fun getGravDataX(): Short {
//        return combineMSBandLSB(
//            readShort(Register.GRV_DATA_X_MSB),
//            readShort(Register.GRV_DATA_X_LSB))
//    }
//    fun getLinearAccDataZ(): Double {
//        return combineMSBandLSB(
//            readShort(Register.LIA_DATA_Z_MSB),
//            readShort(Register.LIA_DATA_Z_LSB)) / 100.0
//    }
//    fun getLinearAccDataY(): Double {
//        return combineMSBandLSB(
//            readShort(Register.LIA_DATA_Y_MSB),
//            readShort(Register.LIA_DATA_Y_LSB)) / 100.0
//    }
//    fun getLinearAccDataX(): Double {
//        return combineMSBandLSB(
//            readShort(Register.LIA_DATA_X_MSB),
//            readShort(Register.LIA_DATA_X_LSB)) / 100.0
//    }
//    fun getQuaternionDataZ(): Short {
//        return combineMSBandLSB(
//            readShort(Register.QUAT_DATA_Z_MSB),
//            readShort(Register.QUAT_DATA_Z_LSB))
//    }
//    fun getQuaternionDataY(): Short {
//        return combineMSBandLSB(
//            readShort(Register.QUAT_DATA_Y_MSB),
//            readShort(Register.QUAT_DATA_Y_LSB))
//    }
//    fun getQuaternionDataX(): Short {
//        return combineMSBandLSB(
//            readShort(Register.QUAT_DATA_X_MSB),
//            readShort(Register.QUAT_DATA_X_LSB))
//    }
//    fun getQuaternionDataW(): Short {
//        return combineMSBandLSB(
//            readShort(Register.QUAT_DATA_W_MSB),
//            readShort(Register.QUAT_DATA_W_LSB))
//    }
//    fun getEulDataPitch(): Short {
//        return combineMSBandLSB(
//            readShort(Register.EUL_PITCH_MSB),
//            readShort(Register.EUL_PITCH_LSB))
//    }
//    fun getEulDataRoll(): Short {
//        return combineMSBandLSB(
//            readShort(Register.EUL_ROLL_MSB),
//            readShort(Register.EUL_ROLL_LSB))
//    }
//    fun getEulDataHeading(): Short {
//        return combineMSBandLSB(
//            readShort(Register.EUL_HEADING_MSB),
//            readShort(Register.EUL_HEADING_LSB))
//    }
//    fun getGyrDataZ(): Short {
//        return combineMSBandLSB(
//            readShort(Register.GYR_DATA_Z_MSB),
//            readShort(Register.GYR_DATA_Z_LSB))
//    }
//    fun getGyrDataY(): Short {
//        return combineMSBandLSB(
//            readShort(Register.GYR_DATA_Y_MSB),
//            readShort(Register.GYR_DATA_Y_LSB))
//    }
//    fun getGyrDataX(): Short {
//        return combineMSBandLSB(
//            readShort(Register.GYR_DATA_X_MSB),
//            readShort(Register.GYR_DATA_X_LSB))
//    }
//    fun getMagDataZ(): Short {
//        return combineMSBandLSB(
//            readShort(Register.MAG_DATA_Z_MSB),
//            readShort(Register.MAG_DATA_Z_LSB))
//    }
//    fun getMagDataY(): Short {
//        return combineMSBandLSB(
//            readShort(Register.MAG_DATA_Y_MSB),
//            readShort(Register.MAG_DATA_Y_LSB))
//    }
//    fun getMagDataX(): Short {
//        return combineMSBandLSB(
//            readShort(Register.MAG_DATA_X_MSB),
//            readShort(Register.MAG_DATA_X_LSB))
//    }
//    fun getAccDataZ(): Short {
//        return combineMSBandLSB(
//            readShort(Register.ACC_DATA_Z_MSB),
//            readShort(Register.ACC_DATA_Z_LSB))
//    }
//    fun getAccDataY(): Short {
//        return combineMSBandLSB(
//            readShort(Register.ACC_DATA_Y_MSB),
//            readShort(Register.ACC_DATA_Y_LSB))
//    }
//    fun getAccDataX(): Short {
//        return combineMSBandLSB(
//            readShort(Register.ACC_DATA_X_MSB),
//            readShort(Register.ACC_DATA_X_LSB))
//    }
//
//    private fun checkCalibrationStatus(): Boolean {
//        val status = readShort(Register.CALIB_STAT).toInt() and 0xFF
//        val sysCal = (status shr 6) and 0x03
//        val gyroCal = (status shr 4) and 0x03
//        val accelCal = (status shr 2) and 0x03
//        val magCal = status and 0x03
//        println("SysCal: $sysCal, GyroCal: $gyroCal, AccelCal: $accelCal, MagCal: $magCal")
//        return sysCal == 3 && gyroCal == 3 && accelCal == 3 && magCal == 3
//    }
//
//    fun getNewCalibrationValues() {
//        while(!checkCalibrationStatus()){
//            Thread.sleep(100)
//        }
//
//        val values = shortArrayOf(
//            readShort(Register.MAG_RADIUS_MSB),
//            readShort(Register.MAG_RADIUS_LSB),
//            readShort(Register.ACC_RADIUS_MSB),
//            readShort(Register.ACC_RADIUS_LSB),
//            readShort(Register.GYR_OFFSET_Z_MSB),
//            readShort(Register.GYR_OFFSET_Z_LSB),
//            readShort(Register.GYR_OFFSET_Y_MSB),
//            readShort(Register.GYR_OFFSET_Y_LSB),
//            readShort(Register.GYR_OFFSET_X_MSB),
//            readShort(Register.GYR_OFFSET_X_LSB),
//            readShort(Register.MAG_OFFSET_Z_MSB),
//            readShort(Register.MAG_OFFSET_Z_LSB),
//            readShort(Register.MAG_OFFSET_Y_MSB),
//            readShort(Register.MAG_OFFSET_Y_LSB),
//            readShort(Register.MAG_OFFSET_X_MSB),
//            readShort(Register.MAG_OFFSET_X_LSB),
//            readShort(Register.ACC_OFFSET_Z_MSB),
//            readShort(Register.ACC_OFFSET_Z_LSB),
//            readShort(Register.ACC_OFFSET_Y_MSB),
//            readShort(Register.ACC_OFFSET_Y_LSB),
//            readShort(Register.ACC_OFFSET_X_MSB),
//            readShort(Register.ACC_OFFSET_X_LSB)
//        )
//        print(values)
//        writeCalibrationValues(values)
//    }
//
//    fun writeCalibrationValues(values: ShortArray) {
//        writeShort(Register.MAG_RADIUS_MSB, values[0])
//        writeShort(Register.MAG_RADIUS_LSB, values[1])
//        writeShort(Register.ACC_RADIUS_MSB, values[2])
//        writeShort(Register.ACC_RADIUS_LSB, values[3])
//        writeShort(Register.GYR_OFFSET_Z_MSB, values[4])
//        writeShort(Register.GYR_OFFSET_Z_LSB, values[5])
//        writeShort(Register.GYR_OFFSET_Y_MSB, values[6])
//        writeShort(Register.GYR_OFFSET_Y_LSB, values[7])
//        writeShort(Register.GYR_OFFSET_X_MSB, values[8])
//        writeShort(Register.GYR_OFFSET_X_LSB, values[9])
//        writeShort(Register.MAG_OFFSET_Z_MSB, values[10])
//        writeShort(Register.MAG_OFFSET_Z_LSB, values[11])
//        writeShort(Register.MAG_OFFSET_Y_MSB, values[12])
//        writeShort(Register.MAG_OFFSET_Y_LSB, values[13])
//        writeShort(Register.MAG_OFFSET_X_MSB, values[14])
//        writeShort(Register.MAG_OFFSET_X_LSB, values[15])
//        writeShort(Register.ACC_OFFSET_Z_MSB, values[16])
//        writeShort(Register.ACC_OFFSET_Z_LSB, values[17])
//        writeShort(Register.ACC_OFFSET_Y_MSB, values[18])
//        writeShort(Register.ACC_OFFSET_Y_LSB, values[19])
//        writeShort(Register.ACC_OFFSET_X_MSB, values[20])
//        writeShort(Register.ACC_OFFSET_X_LSB, values[21])
//    }
//
////    fun getMagRadius(): Short {
////        return combineMSBandLSB(
////            readShort(Register.MAG_RADIUS_MSB),
////            readShort(Register.MAG_RADIUS_LSB)
////        )
////    }
////    fun getAccRadius(): Short {
////        return combineMSBandLSB(
////            readShort(Register.ACC_RADIUS_MSB),
////            readShort(Register.ACC_RADIUS_LSB)
////        )
////    }
////    fun getGyrOffsetZ(): Short {
////        return combineMSBandLSB(
////            readShort(Register.GYR_OFFSET_Z_MSB),
////            readShort(Register.GYR_OFFSET_Z_LSB)
////        )
////    }
////    fun getGyrOffsetY(): Short {
////        return combineMSBandLSB(
////            readShort(Register.GYR_OFFSET_Y_MSB),
////            readShort(Register.GYR_OFFSET_Y_LSB)
////        )
////    }
////    fun getGyrOffsetX(): Short {
////        return combineMSBandLSB(
////            readShort(Register.GYR_OFFSET_X_MSB),
////            readShort(Register.GYR_OFFSET_X_LSB)
////        )
////    }
////    fun getMagOffsetZ(): Short {
////        return combineMSBandLSB(
////            readShort(Register.MAG_OFFSET_Z_MSB),
////            readShort(Register.MAG_OFFSET_Z_LSB)
////        )
////    }
////    fun getMagOffsetY(): Short {
////        return combineMSBandLSB(
////            readShort(Register.MAG_OFFSET_Y_MSB),
////            readShort(Register.MAG_OFFSET_Y_LSB)
////        )
////    }
////    fun getMagOffsetX(): Short {
////        return combineMSBandLSB(
////            readShort(Register.MAG_OFFSET_X_MSB),
////            readShort(Register.MAG_OFFSET_X_LSB)
////        )
////    }
////    fun getAccOffsetZ(): Short {
////        return combineMSBandLSB(
////            readShort(Register.ACC_OFFSET_Z_MSB),
////            readShort(Register.ACC_OFFSET_Z_LSB)
////        )
////    }
////    fun getAccOffsetY(): Short {
////        return combineMSBandLSB(
////            readShort(Register.ACC_OFFSET_Y_MSB),
////            readShort(Register.ACC_OFFSET_Y_LSB)
////        )
////    }
////    fun getAccOffsetX(): Short {
////        return combineMSBandLSB(
////            readShort(Register.ACC_OFFSET_X_MSB),
////            readShort(Register.ACC_OFFSET_X_LSB))
////    }
//}