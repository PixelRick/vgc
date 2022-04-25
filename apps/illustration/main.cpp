// Copyright 2021 The VGC Developers
// See the COPYRIGHT file at the top-level directory of this distribution
// and at https://github.com/vgc/vgc/blob/master/COPYRIGHT
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QAbstractNativeEventFilter>
#include <QApplication>
#include <QDir>
#include <QSettings>
#include <QTimer>

#include <vgc/core/paths.h>
#include <vgc/core/python.h>
#include <vgc/dom/document.h>
#include <vgc/widgets/font.h>
#include <vgc/widgets/mainwindow.h>
#include <vgc/widgets/openglviewer.h>
#include <vgc/widgets/qtutil.h>
#include <vgc/widgets/stylesheets.h>

namespace py = pybind11;

class HDInputNativeEventFilter : public QAbstractNativeEventFilter {
public:
    static constexpr bool usePointer = true;

    HDInputNativeEventFilter() {
        if (usePointer) {
            EnableMouseInPointer(true);
        }
    }

    bool nativeEventFilter(const QByteArray &eventType, void *message, long *) override {
        if (eventType == "windows_generic_MSG") {
            MSG *msg = reinterpret_cast<MSG*>(message);
            /*if (msg->message == WM_INPUT)
            {
                UINT dwSize = 40;
                static BYTE lpb[40];
                if(!GetRawInputData((HRAWINPUT)msg->lParam, RID_INPUT,lpb, &dwSize, sizeof(RAWINPUTHEADER)))
                    qDebug()<<"Error GetRawInputData";
                else
                {
                    RAWINPUT* raw = (RAWINPUT*)lpb;
                    if (raw->header.dwType == RIM_TYPEMOUSE)
                    {
                        int xPosRelative = raw->data.mouse.lLastX;
                        int yPosRelative = raw->data.mouse.lLastY;

                        QPoint winEventPosition(GET_X_LPARAM(msg.lParam), GET_Y_LPARAM(msg.lParam));
                    }
                }

            }*/
            if (msg->message == WM_POINTERUPDATE) {
                const uint32_t pointerId = GET_POINTERID_WPARAM(msg->wParam);

                POINTER_INPUT_TYPE pointerType;
                GetPointerType(pointerId, &pointerType);
                if (pointerType == PT_MOUSE) {
                    POINTER_INFO info;
                    GetPointerInfo(pointerId, &info);
                    OutputDebugString(vgc::core::format("Pointer Info: history count {}\n", (int)info.historyCount).c_str());
                }

            }
            else if (msg->message == WM_MOUSEMOVE) {
                if (usePointer || isSendingMM) {// || (GetAsyncKeyState(0x4A) & 0x8000)) {
                    return false;
                }

                auto pt = MAKEPOINTS(msg->lParam);

                // snippet from https://docs.microsoft.com/en-us/windows/win32/api/winuser/nf-winuser-getmousemovepointsex
                int nVirtualWidth = GetSystemMetrics(SM_CXVIRTUALSCREEN) ;
                int nVirtualHeight = GetSystemMetrics(SM_CYVIRTUALSCREEN) ;
                int nVirtualLeft = GetSystemMetrics(SM_XVIRTUALSCREEN) ;
                int nVirtualTop = GetSystemMetrics(SM_YVIRTUALSCREEN) ;
                int cpt = 0 ;
                // position is in virtual desktop space
                int mode = GMMP_USE_DISPLAY_POINTS;
                // position is weird..
                //int mode = GMMP_USE_HIGH_RESOLUTION_POINTS;

                MOUSEMOVEPOINT mp_in ;
                MOUSEMOVEPOINT mp_out[64] ;

                POINT pos;
                GetCursorPos(&pos);

                ZeroMemory(&mp_in, sizeof(mp_in)) ;
                //mp_in.x = pt.x & 0x0000FFFF ; // Ensure that this number will pass through.
                //mp_in.y = pt.y & 0x0000FFFF ;
                mp_in.x = pos.x & 0x0000FFFF ;
                mp_in.y = pos.y & 0x0000FFFF ;

                //mp_in.time = msg->time;

                cpt = GetMouseMovePointsEx(sizeof(mp_in), &mp_in, mp_out, 64, mode) ;
                if (cpt < 0) {
                    DWORD errorMessageID = ::GetLastError();
                    OutputDebugString(vgc::core::format("GetMouseMovePointsEx: failed, err is {}\n", (int)errorMessageID).c_str());
                    return false;
                }
                //OutputDebugString(vgc::core::format("GetMouseMovePointsEx: ok, count is {}, MM time is {}\n", cpt, msg->time).c_str());

                DWORD mostRecentValidTime = lastMMTime;
                isSendingMM = true;
                for (int j = 0; j < cpt; j++) {
                    int i = cpt - 1 - j; // go the other way..

                    // XXX do something when time == lastTime
                    //  for instance find last that match in position
                    // XXX support time overflow
                    if (mp_out[i].time <= lastMMTime) {
                        //OutputDebugString(vgc::core::format("GetMouseMovePointsEx: skipping old point at index {} (timestamp:{})\n", i, mp_out[i].time).c_str());
                        continue;
                    }

                    switch(mode) {
                    case GMMP_USE_DISPLAY_POINTS:
                        if (mp_out[i].x > 32767)
                            mp_out[i].x -= 65536;
                        if (mp_out[i].y > 32767)
                            mp_out[i].y -= 65536;
                        break ;
                    case GMMP_USE_HIGH_RESOLUTION_POINTS:
                        mp_out[i].x = ((mp_out[i].x * (nVirtualWidth - 1)) - (nVirtualLeft * 65536)) / nVirtualWidth;
                        mp_out[i].y = ((mp_out[i].y * (nVirtualHeight - 1)) - (nVirtualTop * 65536)) / nVirtualHeight;
                        break ;
                    }

                    int x = mp_out[i].x;
                    int y = mp_out[i].y;

                    if ((x < 0 || x > 0x0000FFFF) || (y < 0 || y > 0x0000FFFF)) {
                        //OutputDebugString(vgc::core::format("GetMouseMovePointsEx: invalid point at index {}, (x:{}, y:{})\n", i, x, y).c_str());
                        continue;
                    }

                    mostRecentValidTime = mp_out[i].time;
                    //OutputDebugString(vgc::core::format("GetMouseMovePointsEx: sending historical point (index:{}, timestamp:{})\n", i, mp_out[i].time).c_str());
                    auto lParam = MAKELPARAM(x, y);
                    SendMessage(msg->hwnd, WM_MOUSEMOVE, msg->wParam, lParam);
                }
                isSendingMM = false;
                lastMMTime = mostRecentValidTime;
                return true;
            }
        }
        return false;
    }

    DWORD lastMMTime = 0;
    bool isSendingMM = false;
};
//
//class HDInputEventFilter : public QObject {
//public:
//    bool eventFilter(QObject *obj, QEvent *event) override {
//        if (!recursing && event->type() == QEvent::MouseMove) {
//
//        }
//        return QObject::eventFilter(obj, event);
//    }
//
//    bool recursing = false;
//};


int main(int argc, char* argv[])
{
    // Conversion between QString and std::string.
    using vgc::widgets::fromQt;
    using vgc::widgets::toQt;

    // Init OpenGL. Must be called before QApplication creation. See Qt doc:
    //
    // Calling QSurfaceFormat::setDefaultFormat() before constructing the
    // QApplication instance is mandatory on some platforms (for example,
    // macOS) when an OpenGL core profile context is requested. This is to
    // ensure that resource sharing between contexts stays functional as all
    // internal contexts are created using the correct version and profile.
    //
    vgc::widgets::OpenGLViewer::init();

    // Creates the QApplication
    // XXX We should create a vgc::???::Application class for code sharing
    // between the different VGC apps.
    QApplication application(argc, argv);

    HDInputNativeEventFilter filter;
    application.installNativeEventFilter(&filter);

    // Set runtime paths from vgc.conf, an optional configuration file to be
    // placed in the same folder as the executable.
    //
    // If vgc.conf exists, then the specified paths can be either absolute or
    // relative to the directory where vgc.conf lives (that is, relative to the
    // application dir path).
    //
    // If vgc.conf does not exist, or BasePath isn't specified, then BasePath
    // is assumed to be ".." (that is, one directory above the application dir
    // path).
    //
    // If vgc.conf does not exist, or PythonHome isn't specified, then
    // PythonHome is assumed to be equal to BasePath.
    //
    // Note: in the future, we would probably want this to be handled directly
    // by vgc::core, for example via a function vgc::core::init(argc, argv).
    // For now, we keep it here for the convenience of being able to use Qt's
    // applicationDirPath(), QDir, and QSettings. We don't want vgc::core to
    // depend on Qt.
    //
    QString binPath = QCoreApplication::applicationDirPath();
    QDir binDir(binPath);
    binDir.makeAbsolute();
    binDir.setPath(binDir.canonicalPath()); // resolve symlinks
    QDir baseDir = binDir;
    baseDir.cdUp();
    std::string basePath = fromQt(baseDir.path());
    std::string pythonHome = basePath;
    if (binDir.exists("vgc.conf")) {
        QSettings conf(binDir.filePath("vgc.conf"), QSettings::IniFormat);
        if (conf.contains("BasePath")) {
            QString v = conf.value("BasePath").toString();
            if (!v.isEmpty()) {
                v = QDir::cleanPath(binDir.filePath(v));
                basePath = fromQt(v);
                pythonHome = fromQt(v);
            }
        }
        if (conf.contains("PythonHome")) {
            QString v = conf.value("PythonHome").toString();
            if (!v.isEmpty()) {
                v = QDir::cleanPath(binDir.filePath(v));
                pythonHome = fromQt(v);
            }
        }
    }
    vgc::core::setBasePath(basePath);

    // Create the python interpreter
    std::string programName(argv[0]);
    vgc::core::PythonInterpreter pythonInterpreter(programName, pythonHome);

    // Create the document + root element
    // -> Let's have the MainWindow be the owner of the document for now.
    //    Later, it should be the VgcIllustrationApp, accessible from the
    //    MainWindow so that it could call app->setDocument(doc) on open.
    //auto doc = vgc::dom::Document::create();
    //vgc::dom::Element::create(doc.get(), "vgc");

    // Expose the Document instance to the Python console as a local Python
    // variable 'document'.
    //
    // XXX In the long term, we may not want to expose "document" directly, but:
    // 1. Have a class VgcIllustrationApp: public QApplication.
    // 2. Have an instance 'VgcIllustrationApp app'.
    // 3. Pass the app to python.
    // 4. Users can call things like:
    //      app.document() (or just app.document, which is more pythonic)
    //      app.currentDocument()
    //      app.documents()
    //      etc.
    //
    // One advantage is that the calls above can be made read-only.
    // Currently, users can do document = Document() and then are not able
    // to affect the actual document anymore...
    //
    //pythonInterpreter.run("import vgc.dom");
    //pythonInterpreter.setVariableValue("document", document);

    // Create the main window
    //vgc::widgets::MainWindow w(doc.get(), &pythonInterpreter);
    vgc::widgets::MainWindow w(&pythonInterpreter);
    w.setWindowTitle("VGC Illustration");

    // Set style
    vgc::widgets::addDefaultApplicationFonts();
    vgc::widgets::setApplicationStyleSheet("widgets/stylesheets/dark.qss");

    // Set window icon
    std::string iconPath = vgc::core::resourcePath("apps/illustration/icons/512.png");
    application.setWindowIcon(QIcon(toQt(iconPath)));

    // Show maximized.
    //
    // We must call showMaximized() after the event loop has started,
    // otherwise the QMenuBar's background won't extend to the full length of
    // the window. This is a known Qt bug:
    //
    //   https://bugreports.qt.io/browse/QTBUG-55690
    //
    QTimer timer;
    timer.setSingleShot(true);
    QObject::connect(&timer, SIGNAL(timeout()), &w, SLOT(showMaximized()));
    timer.start(10);

    // Start event loop
    return application.exec();
}
