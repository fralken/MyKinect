using LightBuzz.Vitruvius;
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;

namespace MyKinect
{
    class KinectManager
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor _kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper _coordinateMapper = null;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader _multiFrameSourceReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private GreenScreenBitmapGenerator _bitmap = null;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup _drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage _drawImage;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float _InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush _handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush _handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush _handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double _HandSize = 30;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double _ClipBoundsThickness = 10;

        public EventHandler<EventArgs> GuestHandlerBefore = null;
        public EventHandler<EventArgs> GuestHandlerAfter = null;

        public KinectManager()
        {
            _kinectSensor = KinectSensor.GetDefault();

            _multiFrameSourceReader = _kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.BodyIndex | FrameSourceTypes.Body);

            _multiFrameSourceReader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

            _coordinateMapper = _kinectSensor.CoordinateMapper;

            _bitmap = GreenScreenBitmapGenerator.Create(_coordinateMapper, false);

            // Create the drawing group we'll use for drawing
            _drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            _drawImage = new DrawingImage(_drawingGroup);

            _kinectSensor.Open();
        }

        public ImageSource getBitmap()
        {
            return _bitmap.Bitmap;
        }

        public ImageSource getDrawImage()
        {
            return _drawImage;
        }

        public void Dispose()
        {
            if (_multiFrameSourceReader != null)
            {
                // MultiSourceFrameReder is IDisposable
                _multiFrameSourceReader.Dispose();
                _multiFrameSourceReader = null;
            }

            if (_kinectSensor != null)
            {
                _kinectSensor.Close();
                _kinectSensor = null;
            }
        }


        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            if (GuestHandlerBefore != null)
                GuestHandlerBefore(sender, e);

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            using (var depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            using (var colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            using (var bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame())
            using (var bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {

                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if ((depthFrame == null) || (colorFrame == null) || (bodyIndexFrame == null) || (bodyFrame == null))
                {
                    return;
                }

                _bitmap.Update(colorFrame, depthFrame, bodyIndexFrame);

                // bodyFrame
                var bodies = bodyFrame.Bodies();

                using (DrawingContext dc = _drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    //dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, _depthWidth, _depthHeight));
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, _bitmap.Bitmap.Width, _bitmap.Bitmap.Height));

                    foreach (Body body in bodies)
                    {
                        if (body.IsTracked)
                        {
                            DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = _InferredZPositionClamp;
                                }

                                if (_bitmap.isHD)
                                {
                                    ColorSpacePoint colorSpacePoint = _coordinateMapper.MapCameraPointToColorSpace(position);
                                    jointPoints[jointType] = new Point(colorSpacePoint.X, colorSpacePoint.Y);
                                }
                                else
                                {
                                    DepthSpacePoint depthSpacePoint = _coordinateMapper.MapCameraPointToDepthSpace(position);
                                    jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                }
                            }

                            DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    //_drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, _depthWidth, _depthHeight));
                    _drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, _bitmap.Bitmap.Width, _bitmap.Bitmap.Height));
                }
            }

            if (GuestHandlerAfter != null)
                GuestHandlerAfter(sender, e);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(_handClosedBrush, null, handPosition, _HandSize, _HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(_handOpenBrush, null, handPosition, _HandSize, _HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(_handLassoBrush, null, handPosition, _HandSize, _HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            double width = _bitmap.Bitmap.Width;
            double height = _bitmap.Bitmap.Height;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, height - _ClipBoundsThickness, width, _ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, width, _ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, _ClipBoundsThickness, height));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(width - _ClipBoundsThickness, 0, _ClipBoundsThickness, height));
            }
        }
    }
}
