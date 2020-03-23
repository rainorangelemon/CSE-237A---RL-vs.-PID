import RPi.GPIO as GPIO
from calculate_angles import get_rotation
from time import sleep
from time import time
import numpy as np
import torch
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F
import math

HID_SIZE = 10
DT = 0.035
LEARNING_RATE = 0.001
gamma = 0.5
ENTROPY_BETA = 10  # encourage exploration
decay_ratio = 0.98


class QNet(nn.Module):
    def __init__(self, obs_size, act_size):
        super(QNet, self).__init__()

        self.base = nn.Sequential(
            nn.Linear(obs_size, HID_SIZE),
            nn.ReLU(),
        )
        self.policy = nn.Sequential(
            nn.Linear(HID_SIZE, act_size),
            nn.Softmax(),
        )
        self.value = nn.Sequential(
            nn.Linear(HID_SIZE, 1),
        )

    def forward(self, x):
        base_out = self.base(x)
        return self.policy(base_out),  self.value(base_out)


model = QNet(2, 21)
optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)
# comment it if no such file
model.load_state_dict(torch.load('model_best.pth.tar'))

GPIO.setmode(GPIO.BCM)

GPIO.setup(0, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)

GPIO.setup(11, GPIO.IN)  # button

GPIO.output(5, GPIO.HIGH)
GPIO.output(6, GPIO.LOW)
GPIO.output(13, GPIO.HIGH)
GPIO.output(19, GPIO.LOW)

motor1 = GPIO.PWM(0, 101)
motor2 = GPIO.PWM(26, 101)

motor1.start(0)
motor2.start(0)

state = prev_button = False
experience = []


def control_motor(choice):
    speed = -100 + 10 * choice
    if speed < 0:
        GPIO.output(5, GPIO.HIGH)
        GPIO.output(6, GPIO.LOW)
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(19, GPIO.LOW)
    else:
        GPIO.output(5, GPIO.LOW)
        GPIO.output(6, GPIO.HIGH)
        GPIO.output(13, GPIO.LOW)
        GPIO.output(19, GPIO.HIGH)
    motor1.ChangeDutyCycle(abs(speed))
    motor2.ChangeDutyCycle(abs(speed))


def get_reward(angle):
    if abs(angle) < 10:
        return 1
    if abs(angle) > 20:
        return -1
    else:
        return 0


def calc_logprob(policy_v, actions_v):
    p = - torch.log(policy_v.gather(1, actions_v.view(-1, 1)))
    return p


def train(states, actions_):
    if len(actions_) <= 1:
        return

    # prepare inputs
    values = []
    advs = []
    for i in range(len(states)):
        if i == (len(states) - 1):
            reward = get_reward(states[i][-1])
        else:
            next_angle = states[i+1][-1]
            reward = get_reward(next_angle) + gamma * model(torch.from_numpy(np.array(states[i+1]).astype(np.float32)))[1].detach().numpy()[0]
        values.append(reward)
        advs.append(reward - model(torch.from_numpy(np.array(states[i]).astype(np.float32)))[1].detach().numpy()[0])
    states, values = torch.from_numpy(np.array(states).astype(np.float32)), torch.from_numpy(np.array(values).astype(np.float32))
    advs = torch.from_numpy(np.array(advs).astype(np.float32))

    # experience.append((advs, states, values, actions_))
    #
    # if len(experience) > 100:
    #     experience.pop(0)
    #
    # for advs, states, values, actions in experience:
        # start training

    optimizer.zero_grad()
    policy_, value_ = model(states)
    ids = torch.Tensor(actions).long()
    log_prob_v = advs * calc_logprob(policy_, ids).squeeze(-1)
    loss_policy_v = -log_prob_v.mean()
    m = torch.distributions.Categorical(policy_)
    entropy_loss_v = ENTROPY_BETA * m.entropy().mean()

    value_loss_v = F.mse_loss(value_.squeeze(-1), values)

    print(loss_policy_v, - entropy_loss_v, value_loss_v)

    loss_v = loss_policy_v - entropy_loss_v + value_loss_v
    loss_v.backward()
    optimizer.step()
    torch.save(model.state_dict(), 'model_best.pth.tar')


states = []
actions = []
trained = False

try:

    while True:

        try:

            val = GPIO.input(11)
            if val == GPIO.LOW:
                if not prev_button:
                    current_time = time()

                    state = not state
                    prev_button = True
            else:
                prev_button = False

            if not state:
                motor1.ChangeDutyCycle(0)
                motor2.ChangeDutyCycle(0)
                if not trained:
                    train(states, actions)
                    states = []
                    actions = []
                    trained = True
                    print('trained!')
                continue

            current_time = time()

            # RL learning

            y_gyro, y_angle = get_rotation()

            if abs(y_angle) > 70:
                state = False
                trained = False

            policy, _ = model(torch.from_numpy(np.array([y_gyro, y_angle]).astype(np.float32)))
            policy = policy.detach().numpy()
            choice = np.random.choice(21, 1, p=policy)[0]
            control_motor(choice)

            states.append((y_gyro, y_angle))
            actions.append(choice)
            # print(y_angle, choice, time() - current_time)

            sleep(DT - (time() - current_time))

        except Exception as e:
            print(e)

            states = []
            actions = []
            state = False
            trained = False

        finally:
            #  do nothing
            m = 1

finally:
    GPIO.cleanup()
