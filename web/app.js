const { createApp } = Vue;

// Joystick Pad Component
const JoystickPad = {
  props: {
    side: {
      type: String,
      required: true,
      validator: value => ['left', 'right'].includes(value)
    },
    isActive: Boolean,
    isInactive: Boolean
  },
  emits: ['update-state', 'pad-down', 'pad-up'],
  data() {
    return {
      state: { x: 0, y: 0 },
      pointerId: null,
      knobPosition: { left: '50%', top: '50%' }
    };
  },
  computed: {
    padClasses() {
      return {
        pad: true,
        active: this.isActive,
        inactive: this.isInactive
      };
    }
  },
  methods: {
    updateKnob(nx, ny) {
      this.knobPosition = {
        left: (50 + nx * 50) + '%',
        top: (50 - ny * 50) + '%'
      };
    },
    down(e) {
      e.preventDefault();
      this.$emit('pad-down', this.side);
      this.pointerId = e.pointerId || 1;
      this.move(e);
    },
    up(e) {
      e.preventDefault();
      if (!this.isActive) return;

      this.pointerId = null;
      this.$emit('pad-up', this.side);
      this.state = { x: 0, y: 0 };
      this.updateKnob(0, 0);
      this.$emit('update-state', this.state);
    },
    move(e) {
      if (!this.isActive) return;
      if (this.pointerId !== null && e.pointerId && e.pointerId !== this.pointerId) return;

      const pad = this.$refs.pad;
      const r = pad.getBoundingClientRect();
      let px = e.clientX || (e.touches && e.touches[0] && e.touches[0].clientX) || 0;
      let py = e.clientY || (e.touches && e.touches[0] && e.touches[0].clientY) || 0;
      let dx = px - (r.left + r.width / 2);
      let dy = py - (r.top + r.height / 2);
      let maxR = Math.min(r.width, r.height) / 2;
      let nx = Math.max(-1, Math.min(1, dx / maxR));
      let ny = Math.max(-1, Math.min(1, -dy / maxR)); // Invert Y axis

      this.state = { x: nx, y: ny };
      this.updateKnob(nx, ny);
      this.$emit('update-state', this.state);
    }
  },
  mounted() {
    const pad = this.$refs.pad;

    // Pointer events
    pad.addEventListener('pointerdown', this.down);
    pad.addEventListener('pointermove', this.move);
    pad.addEventListener('pointerup', this.up);
    pad.addEventListener('pointercancel', this.up);

    // Touch events for better mobile support
    pad.addEventListener('touchstart', this.down, { passive: false });
    pad.addEventListener('touchmove', this.move, { passive: false });
    pad.addEventListener('touchend', this.up, { passive: false });
  },
  template: `
    <div ref="pad" :class="padClasses">
      <div class="knob" :style="knobPosition"></div>
    </div>
  `
};

// Control Button Component
const ControlButton = {
  props: {
    label: String,
    active: Boolean
  },
  emits: ['click'],
  computed: {
    buttonClasses() {
      return {
        btn: true,
        active: this.active
      };
    }
  },
  methods: {
    handleClick() {
      this.$emit('click');
    }
  },
  template: `
    <button :class="buttonClasses" @click="handleClick">
      {{ label }}
    </button>
  `
};

// WebSocket Service
class WebSocketService {
  constructor() {
    this.ws = null;
    this.connected = false;
    this.reconnectInterval = null;
    this.onStatusUpdate = null;
    this.onConnectionChange = null;
  }

  connect() {
    try {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const wsUrl = `${protocol}//${window.location.host}/`;
      this.ws = new WebSocket(wsUrl);

      this.ws.onopen = () => {
        console.log('WebSocket connected');
        this.connected = true;
        if (this.onConnectionChange) {
          this.onConnectionChange(true, 'Connected via WebSocket');
        }
        if (this.reconnectInterval) {
          clearInterval(this.reconnectInterval);
          this.reconnectInterval = null;
        }
      };

      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          if (data.type === 'status' && this.onStatusUpdate) {
            this.onStatusUpdate(data);
          }
        } catch (e) {
          console.error('Error parsing WebSocket message:', e);
        }
      };

      this.ws.onclose = () => {
        console.log('WebSocket disconnected');
        this.connected = false;
        if (this.onConnectionChange) {
          this.onConnectionChange(false, 'Disconnected - Reconnecting...');
        }
        if (!this.reconnectInterval) {
          this.reconnectInterval = setInterval(() => this.connect(), 2000);
        }
      };

      this.ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        if (this.onConnectionChange) {
          this.onConnectionChange(false, 'Connection error');
        }
      };

    } catch (e) {
      console.error('Error creating WebSocket:', e);
      if (this.onConnectionChange) {
        this.onConnectionChange(false, 'WebSocket not supported');
      }
    }
  }

  send(data) {
    if (this.ws && this.connected && this.ws.readyState === WebSocket.OPEN) {
      try {
        this.ws.send(JSON.stringify(data));
        return true;
      } catch (e) {
        console.error('Error sending WebSocket data:', e);
        this.connected = false;
        return false;
      }
    }
    return false;
  }

  disconnect() {
    if (this.reconnectInterval) {
      clearInterval(this.reconnectInterval);
      this.reconnectInterval = null;
    }
    if (this.ws) {
      this.ws.close();
    }
  }
}

// Main Application Component
const SwerveDriveApp = {
  components: {
    JoystickPad,
    ControlButton
  },
  props: {
    ssid: {
      type: String,
      default: 'Disconnected'
    }
  },
  data() {
    return {
      leftState: { x: 0, y: 0 },
      rightState: { x: 0, y: 0 },
      activePad: null,
      joystickButtons: { left: false, right: false },
      buttons: { home: false, demo: false },
      status: 'Connecting...',
      wsInfo: 'Initializing...',
      wsService: null,
      sendTimer: null,
      updateIntervalMs: 100
    };
  },
  computed: {
    leftPadActive() {
      return this.activePad === 'left';
    },
    leftPadInactive() {
      return this.activePad === 'right';
    },
    rightPadActive() {
      return this.activePad === 'right';
    },
    rightPadInactive() {
      return this.activePad === 'left';
    }
  },
  methods: {
    onLeftStateUpdate(state) {
      this.leftState = state;
    },
    onRightStateUpdate(state) {
      this.rightState = state;
    },
    onPadDown(side) {
      this.activePad = side;
      this.joystickButtons[side] = true;
    },
    onPadUp(side) {
      if (this.activePad === side) {
        this.activePad = null;
        this.joystickButtons[side] = false;
      }
    },
    onHomeClick() {
      this.buttons.home = !this.buttons.home;
    },
    onDemoClick() {
      this.buttons.demo = !this.buttons.demo;
    },
    sendLoop() {
      const allButtons = {
        ...this.buttons,
        leftJoystick: this.joystickButtons.left,
        rightJoystick: this.joystickButtons.right
      };

      const payload = {
        left: this.leftState,
        right: this.rightState,
        buttons: allButtons
      };

      // Reset momentary buttons
      this.buttons.home = false;
      this.buttons.demo = false;

      const success = this.wsService.send(payload);
      if (!success && this.wsService.connected) {
        this.status = 'Send error - Reconnecting...';
      }

      if (this.sendTimer) clearTimeout(this.sendTimer);
      this.sendTimer = setTimeout(() => this.sendLoop(), this.updateIntervalMs);
    },
    startSendLoop() {
      if (this.sendTimer) clearTimeout(this.sendTimer);
      this.sendTimer = setTimeout(() => this.sendLoop(), this.updateIntervalMs);
    },
    stopSendLoop() {
      if (this.sendTimer) {
        clearTimeout(this.sendTimer);
        this.sendTimer = null;
      }
    }
  },
  mounted() {
    // Initialize WebSocket service
    this.wsService = new WebSocketService();

    this.wsService.onConnectionChange = (connected, message) => {
      this.status = `${message}`;
      this.wsInfo = `${connected ? 'Connected' : 'Disconnected'}`;
      if (connected) {
        this.startSendLoop();
      }
    };

    this.wsService.onStatusUpdate = (data) => {
      this.status = `Connected (${data.connected_clients} clients) - Mode: ${data.mode}`;
    };

    this.wsService.connect();
  },
  beforeUnmount() {
    this.stopSendLoop();
    if (this.wsService) {
      this.wsService.disconnect();
    }
  },
  template: `
    <div class="container">
      <h2>Mini SwerveDrive</h2>
      <div class="row">
        <joystick-pad 
          side="left"
          :is-active="leftPadActive"
          :is-inactive="leftPadInactive"
          @update-state="onLeftStateUpdate"
          @pad-down="onPadDown"
          @pad-up="onPadUp"
        ></joystick-pad>
        <joystick-pad 
          side="right"
          :is-active="rightPadActive"
          :is-inactive="rightPadInactive"
          @update-state="onRightStateUpdate"
          @pad-down="onPadDown"
          @pad-up="onPadUp"
        ></joystick-pad>
      </div>
      <div class="row" id="buttons">
        <control-button 
          label="Home" 
          :active="buttons.home"
          @click="onHomeClick"
        ></control-button>
        <control-button 
          label="Demo" 
          :active="buttons.demo"
          @click="onDemoClick"
        ></control-button>
      </div>
      <div class="label small">Wifi connected: <span class="bold">{{ ssid }}</span></div>
      <div class="label small">Server: <span class="bold">{{ wsInfo }}</span></div>
    </div>
  `
};

// Create and mount the Vue application
createApp({
  components: {
    SwerveDriveApp
  }
}).mount('#app');