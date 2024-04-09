//package org.firstinspires.ftc.teamcode.auto;
//
//@Config
//public abstract class PedroBase extends CommandOpMode {
//    protected final ScrappyConstants.AllianceType m_allianceType;
//    protected final ScrappyConstants.AllianceSide m_allianceSide;
//    public PropDetector.DetectionResult detectionResult = PropDetector.DetectionResult.LEFT;
//    public ScrappyCore robot;
//    public WebcamName webcam1, webcam2;
//    public Pose2d m_startPose;
//
//    protected PropDetectionProcessor propDetectionProcessor;
//    protected AprilTagProcessor aprilTagProcessor;
//    public StackProcessor stackProcessor;
//    public VisionPortal vision;
//    private ElapsedTime elapsedTime = new ElapsedTime();
//
//    public PedroBase(ScrappyConstants.AllianceType allianceType, ScrappyConstants.AllianceSide allianceSide, Pose2d startPose) {
//        m_allianceType = allianceType;
//        m_allianceSide = allianceSide;
//        m_startPose = startPose;
//    }
//
//    @Override
//    public void initialize() {
//        if (!ScrappyConstants.IS_COMPETITION) {
//            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        }
//
//        robot = new ScrappyCore(hardwareMap, m_allianceType, m_allianceSide);
//        robot.m_drive.setStartingPose(m_startPose);
//        new InitPositions(robot.m_outtake, robot.m_intake).schedule();
//
//        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);
//
//        propDetectionProcessor = new PropDetectionProcessor(m_allianceType, m_allianceSide);
//        stackProcessor = new StackProcessor();
//        aprilTagProcessor = new AprilTagProcessor.Builder().build();
//        vision = new VisionPortal.Builder()
//                .setCamera(switchableCamera)
//                .setCameraResolution(new Size(640, 480))
//                .enableLiveView(true)
//                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
//                .addProcessors(stackProcessor, propDetectionProcessor, aprilTagProcessor)
//                .build();
//
//        while (vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            telemetry.addLine("Waiting for cameras to be ready...");
//            telemetry.update();
//            sleep(10);
//        }
//
//        vision.setActiveCamera(webcam2);
//        vision.setProcessorEnabled(stackProcessor, false);
//        vision.setProcessorEnabled(aprilTagProcessor, false);
//        vision.setProcessorEnabled(propDetectionProcessor, true);
//
//        telemetry.addLine("Initializing trajectories...");
//        telemetry.update();
//        initAuto();
//
//        while (opModeInInit()) {
//            detectionResult = propDetectionProcessor.getDetectionResult();
//            telemetry.addData("Detected", detectionResult);
//            telemetry.update();
//            sleep(25);
//        }
//
//        vision.setProcessorEnabled(propDetectionProcessor, false);
//
//        if (m_allianceSide == ScrappyConstants.AllianceSide.FAR) {
//            vision.setProcessorEnabled(aprilTagProcessor, true);
//        } else {
//            vision.setProcessorEnabled(stackProcessor, true);
//        }
//
//        startAuto();
//    }
//
//    @Override
//    public void run() {
//        CommandScheduler.getInstance().run();
//        robot.m_drive.update();
//
////        Pose2d currentPose = robot.m_drive.getPose();
//
////        telemetry.addData("hz", 1000 / elapsedTime.milliseconds());
////        telemetry.addData("x", currentPose.getX());
////        telemetry.addData("y", currentPose.getY());
////        telemetry.addData("heading", currentPose.getHeading());
////        telemetry.addData("velMag", robot.m_drive.getVelocityMagnitude());
//        telemetry.update();
//        elapsedTime.reset();
//    }
//
//    public abstract void initAuto();
//    public abstract void startAuto();
//}