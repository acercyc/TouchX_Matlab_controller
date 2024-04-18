//---------------------------------------------------------------------
//
// Copyright 2009, SensAble Technologies, Inc.
//
// File: QtGlWidget.h
//
// Author: Venkat Gourishankar	
//
// Created: 09/28/2009
//
// Description: The QtGlWidget class interfaces OpenGL with Qt windows 
// for rendering scenes in the uAPI
//
//---------------------------------------------------------------------
#ifndef QGLWIDGET_H
#define QGLWIDGET_H

#include <QWidget>
#include <QGLWidget>
#include <QtGui>
#include <QtOpenGL>

#include "qtwrapper_global.h"

#include "ExportDef.h"
#include "Globals.h"
#include "Cursor.h"
#include "Sphere.h"
#include "DeviceSpace.h"
#include "QHRenderer.h"

QT_BEGIN_NAMESPACE
class QWidget;
QT_END_NAMESPACE

/*! \brief This a global function to start the rendering loops

This function sets the QuickHaptics rendering engine in motion. The function is invoked after defining all objects, effects and callbacks in the main/WinMain
function*/
void QHAPI qhStart(void);

class QTWRAPPER_EXPORT QHQtWin : virtual public QGLWidget, public QHRenderer 
{
	Q_OBJECT

public:
	QHQtWin(QWidget *parent = 0);
	~QHQtWin();

//	QSize minimumSizeHint() const;
//	QSize sizeHint() const;
	///The zero milli-second timer to process the haptic update.
	///Note: Note that QTimer's accuracy depends on the underlying operating system and hardware. Most platforms support an
	///accuracy of 1 millisecond, but Windows 98 supports only 55. If Qt is unable to deliver the requested number of timer
	///clicks, it will silently discard some.
	QTimer *zeroMsTimer;
	
		/*! \brief This function defines a graphic callback that is invoked on every graphic frame.	

	There can exist only ONE graphic callback function per window. Multiple calls to the preDrawCallback function will schedule 
	the function declared last as the Graphics Callback*/


protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();

private:

	bool m_HapticSpaceFlag;
	void (*m_pGraphicLoopCallbackPointer)(void);
	bool m_InitialWorkspace;
	static bool m_InitialSettings;
	static int m_WindowNumber;
	const char* getErrorCodeName(HDerror errorCode);
	void QtErrorCatcher(void);
	///This function checks if the user has made any changes to the title of the window. And sets that string as the title
	void checkAndSetTitle(void);

public:
	void preDrawCallback(void (*m_pGraphicLoopCallbackPointer)(void));

};

#endif // QGLWIDGET_H