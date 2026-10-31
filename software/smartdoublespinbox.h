#ifndef SMARTDOUBLESPINBOX_H
#define SMARTDOUBLESPINBOX_H

#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QLocale>
#include <QTimer>

class SmartDoubleSpinBox : public QDoubleSpinBox
{
    Q_OBJECT
public:
    explicit SmartDoubleSpinBox(QWidget *parent = nullptr)
        : QDoubleSpinBox(parent)
    {
        setLocale(QLocale::c());
    }

    QString textFromValue(double val) const override
    {
        QString s = QString::number(val, 'f', decimals());
        if (s.contains('.')) {
            while (s.endsWith('0')) s.chop(1);
            if (s.endsWith('.')) s.chop(1);
        }
        return s;
    }

    double valueFromText(const QString &text) const override
    {
        QString clean = text.trimmed();
        clean.replace(',', '.');
        bool ok = false;
        double v = clean.toDouble(&ok);
        return ok ? v : minimum();
    }

    QValidator::State validate(QString &input, int &pos) const override
    {
        Q_UNUSED(pos);
        QString clean = input.trimmed();
        clean.replace(',', '.');
        if (clean.isEmpty() || clean == "." || clean == "," || clean == "-" || clean == "+")
            return QValidator::Intermediate;

        bool ok = false;
        double v = clean.toDouble(&ok);
        if (!ok)
            return QValidator::Invalid;

        if (v >= minimum() && v <= maximum())
            return QValidator::Acceptable;

        return QValidator::Intermediate;
    }

    void fixup(QString &input) const override
    {
        QString clean = input.trimmed();
        clean.replace(',', '.');
        bool ok = false;
        double v = clean.toDouble(&ok);
        if (ok) {
            if (v < minimum()) v = minimum();
            if (v > maximum()) v = maximum();
            input = textFromValue(v);
        } else {
            input = textFromValue(minimum());
        }
    }

protected:
    void focusInEvent(QFocusEvent *event) override
    {
        QDoubleSpinBox::focusInEvent(event);
        QTimer::singleShot(0, this, &QDoubleSpinBox::selectAll);
    }
};

#endif // SMARTDOUBLESPINBOX_H

