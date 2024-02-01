#include <avr/io.h>

class RotaryEncoder
{
    private:
        int8_t _encStates[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };
        uint8_t _state;
        int8_t _encVal;  
        volatile uint8_t* _inputRegister;
        uint8_t _inputPin0;
        uint8_t _inputPin1;
        volatile uint8_t* _stateRegister;

    public:
      RotaryEncoder()
      {
      }

      RotaryEncoder(
        volatile uint8_t* inputRegister,
        uint8_t inputPin0,
        uint8_t inputPin1,
        volatile uint8_t* stateRegister)
      {
        _inputRegister = inputRegister;
        _inputPin0 = inputPin0;
        _inputPin1 = inputPin1;
        _stateRegister = stateRegister;

        _state = 3;
        _encVal = 0;
      }

      void Scan()
      {
          _state = (_state<<2) & 0x0f;
          uint8_t input = *_inputRegister;
          _state |= bit_is_clear(input, _inputPin1)<<1 | bit_is_clear(input, _inputPin0);

          _encVal += _encStates[_state];

          if(_encVal > 3 ) //four steps forward
          {
              _encVal = 0;

              if (*_stateRegister == 0)
              {
                  return;
              }

              *_stateRegister -=1;
          }
          else if(_encVal < -3 ) //four steps backwards
          {
              _encVal = 0;

              if (*_stateRegister == 127)
              {
                  return;
              }

              *_stateRegister +=1;
          }
      }
};
