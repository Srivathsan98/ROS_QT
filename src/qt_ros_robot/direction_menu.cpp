#include "direction_menu.h"
#include <QPainter>
#include <QMouseEvent>
#include <QtMath>

DirectionMenu::DirectionMenu(QWidget *parent) : QWidget(parent)
{
    setMinimumSize(300, 300);
}

void DirectionMenu::setItems(const QVector<QString> &items)
{
    menuItems = items;
    update();
}

void DirectionMenu::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int radius = qMin(width(), height())/2 - 10;
    QPoint center(width() / 2, height() / 2);

    int count = menuItems.size();
    if(count == 0) return;

    double angleStep = 360.0/count;
    double startAngle = 0;

    for(int i=0; i < count; i++)
    {
        QColor color = (i == hoveredIndex) ? QColor(100, 150, 255) : QColor(200, 200, 200);

        painter.setBrush(color);
        painter.setPen(Qt::black);

        painter.drawPie(center.x() - radius, center.y() - radius, 2*radius, 2*radius, int(startAngle * 16), int(angleStep * 16));

        double midAngle = (startAngle + angleStep / 2) * M_PI / 180.0;
        int textX = center.x() + int(radius * 0.6 * cos(midAngle));
        int textY = center.y() - int(radius * 0.6 * sin(midAngle));

        painter.setPen(Qt::black);
        painter.drawText(QRect(textX - 30, textY - 10, 60, 20), Qt::AlignCenter, menuItems[i]);

        startAngle += angleStep;
    }
}

int DirectionMenu::itemAt(const QPoint &pos) const
{
    if(menuItems.isEmpty()) return -1;

    QPoint center(width() /2, height()/2);
    QPointF diff = pos - center;

    double dist = qSqrt(diff.x() * diff.x() + diff.y() * diff.y());
    int radius = qMin(width(), height())/2 - 10;
    if(dist > radius) return -1;

    double angle = qAtan2(-diff.y(), diff.x()) * 180.0 / M_PI;
    if(angle < 0) angle += 360.0;

    double angleStep = 360.0 / menuItems.size();
    int index = int(angle/angleStep);
    return index;
}

void DirectionMenu::mousePressEvent(QMouseEvent *event)
{
    int index = itemAt(event->pos());
    if(index >= 0 && index < menuItems.size())
    {
        emit clicked(index);
    }
}
