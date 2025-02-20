#include <avr/io.h>

template <typename TInputRegister, typename TOutputRegister>
class ToggleSwitch
{
  private:

    volatile TInputRegister* _inputRegister;
    uint8_t _inputPin;
    volatile TOutputRegister* _outputRegister;
    uint8_t _outputPin;

    // TODO: this can probably be obviated by a more clever use of the pin
    // value vs. the mask.
    bool _state = false;
    bool _isDown = false;

    TInputRegister _inputPinMask;
    TOutputRegister _outputPinMask;

  public:

    ToggleSwitch() {}

    ToggleSwitch(
        volatile TInputRegister* inputRegister,
        uint8_t inputPin,
        volatile uint16_t* outputRegister,
        uint8_t outputPin)
    {
        _inputRegister = inputRegister;
        _inputPin = inputPin,
        _outputRegister = outputRegister;
        _outputPin = outputPin;

        _inputPinMask = (1 << _inputPin);
        _outputPinMask = (1 << _outputPin);
    }

    bool Scan(bool doSet = true)
    {
        bool isSet = *_inputRegister & _inputPinMask;

        if (_isDown && isSet)
        {
            return isSet;
        }

        _isDown = isSet;

        if (isSet && doSet)
        {
            Set(!_state);
        }

        return isSet;
    }

    bool Get()
    {
        return _state;
    }

    void Set(bool state)
    {
        _state = state;

        if (state)
        {
            *_outputRegister |= _outputPinMask;
        }
        else
        {
            *_outputRegister &= ~_outputPinMask;
        }
    }
};

template <typename TInputRegister, typename TOutputRegister>
class ExclusiveToggleSwitchGroup
{
  private:
    static const int _maxSwitches = 4;

    ToggleSwitch<TInputRegister, TOutputRegister> _switches[_maxSwitches];
    int _switchCount;

  public:
    ExclusiveToggleSwitchGroup() {}

    ExclusiveToggleSwitchGroup(ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch1)
    {
        _switches[0] = toggleSwitch1;

        _switchCount = 1;
    }

    ExclusiveToggleSwitchGroup(
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch1,
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch2)
    {
        _switches[0] = toggleSwitch1;
        _switches[1] = toggleSwitch2;

        _switchCount = 2;
    }

    ExclusiveToggleSwitchGroup(
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch1,
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch2,
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch3)
    {
        _switches[0] = toggleSwitch1;
        _switches[1] = toggleSwitch2;
        _switches[2] = toggleSwitch3;

        _switchCount = 3;
    }

    ExclusiveToggleSwitchGroup(
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch1,
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch2,
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch3,
        ToggleSwitch<TInputRegister, TOutputRegister> toggleSwitch4)
    {
        _switches[0] = toggleSwitch1;
        _switches[1] = toggleSwitch2;
        _switches[2] = toggleSwitch3;
        _switches[3] = toggleSwitch4;

        _switchCount = 4;
    }

    bool Scan()
    {
        // set initial value as 'do-nothing' flag
        int switchIndex = _maxSwitches + 1;

        for (int i = 0; i < _switchCount; i++)
        {
            if (_switches[i].Scan(false))
            {
                switchIndex = i;
                break;
            }
        }

        if (switchIndex > _maxSwitches)
        {
            return false;
        }

        for (int i = 0; i < _switchCount; i++)
        {
            if (i == switchIndex)
            {
                if (_switchCount == 1)
                {
                    _switches[i].Set(1 - _switches[i].Get());
                    continue;
                }

                _switches[i].Set(true);
                continue;
            }

            _switches[i].Set(false);
        }

        return true;
    }
};
