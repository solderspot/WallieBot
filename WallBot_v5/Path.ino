
enum PathState
{
  PTH_WAITING,
  PTH_STOPPING,
  PTH_DONE
};

int16_t *pth_sequence = NULL;
PathState pth_state = PTH_DONE;
//----------------------------------------
//
//----------------------------------------

void init_path( int16_t *sequence )
{
  pth_sequence = sequence;
  pth_next();
  pth_state = PTH_STOPPING;
}

//----------------------------------------
//
//----------------------------------------

void update_path()
{
  switch (pth_state)
  {
    case PTH_WAITING:
      if( pid_service_next())
      {
         pth_state = PTH_STOPPING;      
         stopMotors();
      }
      break;
    case PTH_STOPPING:
      int16_t left, right;
      uint16_t ms;
      if( get_ticks_since_last( &left, &right, &ms) && !left && !right && ms )
      {
        pth_next();
      }
    default:
    case PTH_DONE:
      break;
  }
}

//----------------------------------------
//
//----------------------------------------

void pth_next()
{
  int16_t action; 

  if( !pth_sequence || pth_state == PTH_DONE  )
  {
    return;
  }

  action = *pth_sequence++;

  switch( action )
  {
    case PTH_MOVE:
      pid_move( *pth_sequence++);
      pth_state = PTH_WAITING;
      break;
    case PTH_TURN:
      pid_turn( *pth_sequence++);
      pth_state = PTH_WAITING;
      break;
    default:
    case PTH_END:
      stop();
      pth_state = PTH_DONE;      
      break;
  }
}
