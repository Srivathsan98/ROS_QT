/*
COMPANY     : Cyient Limited.
MODULE      : QT UI Design
SUB MODULE  : Camera Page - PieMenu
DESCRIPTION : This module implemented for the PieMenu. It contains implmentation of pie buttons for panoramic view change.
FUNCTION    : initializeSignalSlotConnections() -> connect the particular slot based on the signals
              setSegment() -> set the selected segment
              getSegment() -> get selected segment using angle
              resizeEvent() -> resize event for current widget.
              paintEvent() -> paint event which draws the pie buttons
              isClickOnLabel() -> checks if there are any clicks on label.
              mousePressEvent() -> handles click events on the pie menu
AUTHOR      : Malhar Sapatnekar
*/

#include "piemenu.h"

PieMenu::PieMenu(QIcon icon, int circleSize, QWidget *parent)
    : QWidget{parent}
    , m_translate(true)
{

    m_circleSize = circleSize;
    setFixedSize(m_circleSize,m_circleSize);
    int centerX = geometry().center().x();
    int centerY = geometry().center().y();
    int labelWidth = 70;
    int labelHeight = 17;
    int labelX = centerX - labelWidth / 2;
    int labelY = centerY - labelHeight / 2;

    m_frontLabel = new QLabel(this);
    m_frontLabel->setGeometry(labelX, labelY-(width()/3), labelWidth, labelHeight);
    m_frontLabel->setStyleSheet(QString("background:none;font-family: 'Roboto';font-style: normal;font-weight: 400;font-size: 14px;line-height: 16px;text-align: center;color: #FFFFFF;"));
    m_frontLabel->setAlignment(Qt::AlignHCenter);
    m_frontLabel->setWordWrap(true);

    m_leftLabel = new QLabel(this);
    m_leftLabel->setGeometry(labelX-(width()/3), labelY, labelWidth, labelHeight);
    m_leftLabel->setStyleSheet(QString("background:none;font-family: 'Roboto';font-style: normal;font-weight: 400;font-size: 14px;line-height: 16px;text-align: center;color: #FFFFFF;"));
    m_leftLabel->setAlignment(Qt::AlignHCenter);

    m_rearLabel = new QLabel(this);
    m_rearLabel->setGeometry(labelX, labelY+(width()/3), labelWidth, labelHeight);
    m_rearLabel->setStyleSheet(QString("background:none;font-family: 'Roboto';font-style: normal;font-weight: 400;font-size: 14px;line-height: 16px;text-align: center;color: #FFFFFF;"));
    m_rearLabel->setAlignment(Qt::AlignHCenter);

    m_rightLabel = new QLabel(this);
    m_rightLabel->setGeometry(labelX+(width()/3), labelY, labelWidth, labelHeight);
    m_rightLabel->setStyleSheet(QString("background:none;font-family: 'Roboto';font-style: normal;font-weight: 400;font-size: 14px;line-height: 16px;text-align: center;color: #FFFFFF;"));
    m_rightLabel->setAlignment(Qt::AlignHCenter);

    m_viewIconLabel = new QLabel(this);
    int iconLabelSize = static_cast<int>((m_circleSize)/2.55);
    m_viewIconLabel->setGeometry(centerX - iconLabelSize/2, centerY-iconLabelSize/2, iconLabelSize, iconLabelSize);
    m_viewIconLabel->setStyleSheet(QString("background-color: black;border-radius:%1px;font-family: 'Roboto';font-style: normal;font-weight: 400;font-size: 14px;line-height: 16px;text-align: center;color: #FFFFFF;").arg(iconLabelSize/2));
    m_viewIconLabel->setPixmap(icon.pixmap(QSize(17, 27)));
    m_viewIconLabel->setAlignment(Qt::AlignCenter);
}

PieMenu::~PieMenu()
{
}

void PieMenu::setSegment(Segment segment)
{
    m_selectedSegment = segment;
}

void PieMenu::updateTranslations()
{
    m_translate = true;
}

void PieMenu::paintEvent(QPaintEvent *event)
{
    if(m_translate){
        m_frontLabel->setText(tr("Front"));
        m_rightLabel->setText(tr("Right"));
        m_leftLabel->setText(tr("Left"));
        m_rearLabel->setText(tr("Rear"));
        m_translate = false;
    }
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    QRect rec = QRect((width() - m_circleSize*0.9) / 2, (height() - m_circleSize*0.9) / 2, m_circleSize*0.9, m_circleSize*0.9);
    setMinimumSize(rec.width(), rec.height());
    int radius = qMin(rec.width(), rec.height()) / 2;
    QPainterPath path;
    path.addEllipse(rec.center(), radius, radius);

    QPainterPath clipPath;
    clipPath.addEllipse(rec.center(), radius/2.5, radius/2.5);
    path = path.subtracted(clipPath);
    int totalSegments = 4;
    int cx = rec.center().x();
    int cy = rec.center().y();

    for(int i = 0; i < totalSegments; i++)
    {
        float angle = static_cast<float>(360.0 / totalSegments * i + 45);
        float cos1 = qCos(qDegreesToRadians(angle));
        float sin1 = qSin(qDegreesToRadians(angle));
        float x1 = static_cast<float>(cx) + static_cast<float>(radius) * cos1;
        float y1 = static_cast<float>(cy) - static_cast<float>(radius) * sin1;
        QPainterPath line;
        line.moveTo(x1 - sin1 , y1 - cos1 );
        line.lineTo(x1 + sin1 , y1 + cos1 );
        line.lineTo(cx, cy);
        path = path.subtracted(line);
    }
    painter.fillPath(path, QColor(0,0,0,102));

    if (segmentAngles.find(m_selectedSegment) != segmentAngles.end()) {
        std::pair<int, int> angles = segmentAngles[m_selectedSegment];
        QPainterPath  SegmentPath;
        SegmentPath.moveTo(cx, cy);
        SegmentPath.arcTo(rec, angles.first,angles.second - angles.first);
        SegmentPath = SegmentPath.subtracted(clipPath);
        painter.fillPath(SegmentPath, QColor(0,0,0,178));

        int newWidth = rec.width() * 0.5;
        int newHeight = rec.height() * 0.5;
        int newX = rec.x() + (rec.width() - newWidth) / 2;
        int newY = rec.y() + (rec.height() - newHeight) / 2;

        QRect smallRect = QRect(newX, newY, newWidth, newHeight);
        QPainterPath smallArcPath;
        smallArcPath.moveTo(cx, cy);
        smallArcPath.arcTo(smallRect, angles.first, angles.second - angles.first);
        smallArcPath = smallArcPath.subtracted(clipPath);
        painter.fillPath(smallArcPath, QColor(255, 146, 0));

        QPoint startPoint(0, 0);
        QPoint midPoint(0, 0);
        QPoint endPoint(0, 0);
        switch (m_selectedSegment) {
        case Segment::TopSegment:
            startPoint = {cx +static_cast<int>(m_circleSize * 0.175) , cy};
            midPoint = {cx - static_cast<int>(m_circleSize * 0.175), cy};
            endPoint = {cx,static_cast<int>(m_circleSize*0.25)};
            break;
        case Segment::LeftSegment:
            startPoint = {cx, cy + static_cast<int>(m_circleSize * 0.175)};
            midPoint = {cx, cy - static_cast<int>(m_circleSize * 0.175)};
            endPoint = {static_cast<int>(m_circleSize*0.25), cy};
            break;
        case Segment::BottomSegment:
            startPoint = {cx + static_cast<int>(m_circleSize * 0.175), cy};
            midPoint = {cx - static_cast<int>(m_circleSize * 0.175), cy};
            endPoint = {cx, static_cast<int>(m_circleSize*0.75)};
            break;
        case Segment::RightSegment:
            startPoint = {cx, cy + static_cast<int>(m_circleSize * 0.175)};
            midPoint = {cx, cy - static_cast<int>(m_circleSize * 0.175)};
            endPoint = {static_cast<int>(m_circleSize*0.75), cy};
            break;
        default:
            break;
        }

        QPainterPath arrowPath;
        arrowPath.moveTo(startPoint);
        arrowPath.lineTo(midPoint);
        arrowPath.lineTo(endPoint);
        arrowPath = arrowPath.subtracted(clipPath);
        painter.fillPath(arrowPath,QColor(255, 146, 0));

    } else {
        qDebug() << "Segment not found in map." ;
    }
}

bool PieMenu::isClickOnLabel(QPoint pos, int centerX, int centerY)
{
    int squaredRadius = m_viewIconLabel->height()/2*m_viewIconLabel->height()/2;
    int dx = static_cast<int>((pos.x() - centerX) * (pos.x() - centerX));
    int dy = static_cast<int>((pos.y() - centerY) * (pos.y() - centerY));
    int squaredDistance = dx + dy;
    return squaredDistance < squaredRadius;
}

void PieMenu::handleButtonClick(Segment segment)
{
    switch (segment) {
    case TopSegment:
        if (m_selectedSegment != Segment::TopSegment) {
            emit topSegmentClicked();
            m_selectedSegment = Segment::TopSegment;
        }
        break;
    case LeftSegment:
        if (m_selectedSegment != Segment::LeftSegment) {
            emit leftSegmentClicked();
            m_selectedSegment = Segment::LeftSegment;
        }
        break;
    case BottomSegment:
        if (m_selectedSegment != Segment::BottomSegment) {
            emit bottomSegmentClicked();
            m_selectedSegment = Segment::BottomSegment;
        }
        break;
    case RightSegment:
        if (m_selectedSegment != Segment::RightSegment) {
            emit rightSegmentClicked();
            m_selectedSegment = Segment::RightSegment;
        }
        break;
    case NoSegment:
        break;
    }
    update();
}

void PieMenu::mousePressEvent(QMouseEvent *event)
{
    QPoint clickPos = event->pos();
    int cx = width() / 2;
    int cy = height() / 2;

    if(isClickOnLabel(clickPos,cx,cy))   return;

    float angle = qRadiansToDegrees(qAtan2(clickPos.y() - cy, clickPos.x() - cx));
    if (angle < 0)
        angle += 360;
    else if (angle > 360)
        angle -= 360;
    float counterClockwiseAngle  = 360 - angle;
    Segment selectedSegment = getSegment(counterClockwiseAngle);
    handleButtonClick(selectedSegment);
    QWidget::mousePressEvent(event);
}

Segment PieMenu::getSegment(float angle) {
    for (const auto& entry : segmentAngles) {
        const auto& range = entry.second;
        if ((angle >= range.first && angle < range.second) ||
            (range.first > range.second && (angle >= range.first || angle < range.second))) {
            return entry.first;
        }
    }
    return RightSegment;
}
