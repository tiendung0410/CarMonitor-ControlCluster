#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include "radialbar.h"
#include "canreceiver.h"  // thêm dòng này
#include <QQmlContext>
#include <QScreen>
#include <QWindow>

int main(int argc, char *argv[])
{
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    
    // Cấu hình cho BeagleBone với màn hình cảm ứng
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
    
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    qmlRegisterType<RadialBar>("CustomControls", 1, 0, "RadialBar");

    CanReceiver canReceiver;
    engine.rootContext()->setContextProperty("canReceiver", &canReceiver);  // expose to QML

    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
        &app, [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        }, Qt::QueuedConnection);
    engine.load(url);

    return app.exec();
}
