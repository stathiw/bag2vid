
#include <QApplication>
#include <QDebug>
#include <QFontDatabase>
#include <QIcon>
#include <QStringList>
#include <QStyleFactory>

#include "bag2vid/frontend/Visualiser.hpp"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    // Fusion is the most QSS-friendly built-in Qt style. Native platform styles
    // (Breeze, Adwaita) can bleed non-themeable chrome — especially in combo popups.
    app.setStyle(QStyleFactory::create("Fusion"));

    const QStringList font_resources = {
        ":/fonts/Fraunces-Variable.ttf",
        ":/fonts/Manrope-Variable.ttf",
        ":/fonts/IBMPlexMono-Regular.ttf",
        ":/fonts/IBMPlexMono-Medium.ttf",
    };
    for (const QString& path : font_resources)
    {
        if (QFontDatabase::addApplicationFont(path) == -1)
        {
            qWarning() << "Failed to load bundled font:" << path;
        }
    }

    QIcon app_icon;
    app_icon.addFile(":/logo/bag2vid-mark-dark-36.png", QSize(36, 36));
    app_icon.addFile(":/logo/bag2vid-mark-dark-64.png", QSize(64, 64));
    app_icon.addFile(":/logo/bag2vid-mark-dark-128.png", QSize(128, 128));
    app_icon.addFile(":/logo/bag2vid-mark-dark-256.png", QSize(256, 256));
    app.setWindowIcon(app_icon);

    bag2vid::Visualiser visualiser;
    visualiser.show();

    return app.exec();
}
