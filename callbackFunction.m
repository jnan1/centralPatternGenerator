function callbackFunction(~, event)
  global mu
  global next_mu
  if strcmp(event.Character, 'h')
    disp(next_mu);
    mu = str2double(next_mu);
    next_mu = '';
    return;
  end
  if strcmp(event.Character, 'c')
      next_mu = '';
      return;
  end
  next_mu = strcat(next_mu, event.Character);
  end
