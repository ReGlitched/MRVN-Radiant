#pragma once

#include "math/vector.h"

class QWidget;
class QMimeData;

void EntityPresetBrowser_Construct();
void EntityPresetBrowser_Destroy();

QWidget* EntityPresetBrowser_constructWindow( QWidget* toplevel );
void EntityPresetBrowser_destroyWindow();

bool EntityPresetBrowser_hasMimeData( const QMimeData* mimeData );
bool EntityPresetBrowser_dropMimeData( const QMimeData* mimeData, const Vector3& origin );
