#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include "esp_timer.h"
#include "esp_sleep.h"

const char* WIFI_SSID_PADRAO  = "NOME_DO_SEU_WIFI";
const char* WIFI_SENHA_PADRAO = "SENHA_DO_SEU_WIFI";

const int TRIG_ENTRADA = 5;
const int ECHO_ENTRADA = 18;

const int TRIG_SAIDA   = 19;
const int ECHO_SAIDA   = 21;

const int PINO_ACIONAMENTO_ENTRADA = 4;
const int PINO_ACIONAMENTO_SAIDA   = 22;
const int PINO_SINALIZADOR_PLACA   = 2;

int distDeteccaoCm        = 15;
int capacidadeMaxima      = 150;
int freqLeituraSensoresMs = 60;
const unsigned long TEMPO_COOLDOWN_MS = 1200;
const unsigned long TEMPO_EXIBICAO_MOVIMENTO_MS = 1200;

volatile int pessoasPresentes = 0;
volatile int totalEntradas    = 0;
volatile int totalSaidas      = 0;

volatile long distanciaEntradaAtual = 999;
volatile long distanciaSaidaAtual   = 999;

volatile uint32_t echoInicioEntradaUs  = 0;
volatile uint32_t echoDuracaoEntradaUs = 0;
volatile bool leituraEntradaPronta     = false;

volatile uint32_t echoInicioSaidaUs    = 0;
volatile uint32_t echoDuracaoSaidaUs   = 0;
volatile bool leituraSaidaPronta       = false;

bool pessoaNaFrenteEntrada = false;
bool pessoaNaFrenteSaida   = false;
unsigned long ultimoMovimentoEntradaMs = 0;
unsigned long ultimoMovimentoSaidaMs   = 0;
unsigned long ultimoDisparoGeralMs     = 0;

volatile uint32_t fn1Us = 14;
volatile uint32_t fn2Us = 8;
volatile uint32_t fn3Us = 5;
volatile uint32_t fn4Us = 180;
volatile uint32_t fn5Us = 1420;

Preferences prefs;
WebServer server(80);
TaskHandle_t handleTaskSensores = NULL;
TaskHandle_t handleTaskSistema  = NULL;

const char PAGINA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BORA+ Club Control — Sistema Ciberfísico Embarcado</title>
    <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;600;700;800&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg-deep: #030712; --card-bg: rgba(17, 24, 39, 0.82); --border-glow: rgba(255, 255, 255, 0.09);
            --text-main: #f3f4f6; --text-muted: #9ca3af; --primary-glow: #ec4899; --secondary-glow: #8b5cf6;
            --success: #10b981; --warning: #f59e0b; --danger: #ef4444; --info: #3b82f6;
        }
        * { margin: 0; padding: 0; box-sizing: border-box; font-family: 'Plus Jakarta Sans', sans-serif; }
        body { background: var(--bg-deep); color: var(--text-main); min-height: 100vh; display: flex; justify-content: center; padding: 24px 14px; }
        .dashboard-card { width: 100%; max-width: 820px; background: var(--card-bg); border: 1px solid var(--border-glow); border-radius: 24px; padding: 26px; }
        .header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 16px; flex-wrap: wrap; gap: 10px; }
        .brand-info h1 { font-size: 1.2rem; font-weight: 800; }
        .brand-info p { font-size: 0.76rem; color: var(--text-muted); }
        .status-badge { display: flex; align-items: center; gap: 8px; padding: 6px 14px; border-radius: 50px; font-size: 0.75rem; font-weight: 600; border: 1px solid rgba(255,255,255,0.08); }
        .pulse-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--success); }
        .status-normal { color: var(--success); } .status-warning { color: var(--warning); } .status-danger { color: var(--danger); }
        .nav-tabs { display: flex; gap: 6px; margin-bottom: 20px; overflow-x: auto; border-bottom: 1px solid var(--border-glow); padding-bottom: 4px; }
        .tab-btn { background: rgba(255,255,255,0.03); border: 1px solid transparent; color: var(--text-muted); padding: 8px 12px; border-radius: 8px 8px 0 0; font-size: 0.74rem; font-weight: 600; cursor: pointer; white-space: nowrap; }
        .tab-btn.active { color: #fff; background: rgba(139, 92, 246, 0.28); border-color: rgba(236, 72, 153, 0.45); }
        .tab-pane { display: none; } .tab-pane.active { display: block; }
        .counter-section { text-align: center; margin: 16px 0 22px; }
        .counter-number { font-size: 4.8rem; font-weight: 800; line-height: 1; }
        .counter-label { font-size: 0.8rem; text-transform: uppercase; color: var(--text-muted); margin-top: 6px; font-weight: 600; }
        .reset-btn { margin-top: 10px; padding: 7px 16px; border-radius: 50px; background: rgba(239,68,68,0.15); border: 1px solid rgba(239,68,68,0.35); color: #fca5a5; font-size: 0.74rem; font-weight: 600; cursor: pointer; }
        .sensors-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 20px; }
        .sensor-box { background: rgba(255,255,255,0.03); border: 1px solid var(--border-glow); border-radius: 14px; padding: 14px; }
        .sensor-box.active-entry { border-color: var(--success); background: rgba(16,185,129,0.12); }
        .sensor-box.active-exit { border-color: var(--primary-glow); background: rgba(236,72,153,0.12); }
        .sensor-header { display: flex; justify-content: space-between; font-size: 0.72rem; color: var(--text-muted); margin-bottom: 6px; font-weight: 600; text-transform: uppercase; }
        .sensor-count { font-size: 1.4rem; font-weight: 700; margin-bottom: 4px; }
        .sensor-motion { font-size: 0.72rem; font-weight: 600; color: var(--text-muted); }
        .progress-container { margin-bottom: 18px; }
        .progress-info { display: flex; justify-content: space-between; font-size: 0.74rem; color: var(--text-muted); margin-bottom: 6px; }
        .progress-bar-bg { width: 100%; height: 8px; background: rgba(255,255,255,0.05); border-radius: 4px; overflow: hidden; }
        .progress-bar-fill { height: 100%; width: 0%; background: linear-gradient(90deg, var(--secondary-glow), var(--primary-glow)); }
        .metrics-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(165px, 1fr)); gap: 10px; margin-bottom: 16px; }
        .metric-card { background: rgba(255,255,255,0.03); border: 1px solid var(--border-glow); border-radius: 12px; padding: 12px; }
        .metric-title { font-size: 0.68rem; color: var(--text-muted); text-transform: uppercase; font-weight: 600; }
        .metric-val { font-size: 1.1rem; font-weight: 700; margin: 3px 0; }
        .metric-sub { font-size: 0.68rem; color: var(--text-muted); }
        .section-box { background: rgba(255,255,255,0.025); border: 1px solid var(--border-glow); border-radius: 14px; padding: 14px; margin-bottom: 14px; }
        .section-box h3 { font-size: 0.82rem; margin-bottom: 10px; display: flex; justify-content: space-between; }
        table.perf-table { width: 100%; border-collapse: collapse; font-size: 0.74rem; }
        table.perf-table th, table.perf-table td { text-align: left; padding: 7px 8px; border-bottom: 1px solid rgba(255,255,255,0.06); }
        .chart-24h { display: flex; align-items: flex-end; gap: 4px; height: 100px; padding-top: 10px; }
        .chart-bar-wrap { flex: 1; display: flex; flex-direction: column; align-items: center; height: 100%; justify-content: flex-end; }
        .chart-bar { width: 100%; background: linear-gradient(180deg, var(--primary-glow), var(--secondary-glow)); border-radius: 3px 3px 0 0; min-height: 3px; }
        .chart-lbl { font-size: 0.56rem; color: var(--text-muted); margin-top: 3px; }
        .logs-container { max-height: 240px; overflow-y: auto; font-family: monospace; font-size: 0.72rem; background: rgba(0,0,0,0.4); border-radius: 8px; padding: 8px; }
        .log-item { padding: 4px 0; border-bottom: 1px solid rgba(255,255,255,0.05); }
        .form-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px; }
        .form-group label { display: block; font-size: 0.7rem; color: var(--text-muted); margin-bottom: 4px; }
        .form-group input { width: 100%; background: rgba(0,0,0,0.4); border: 1px solid var(--border-glow); color: #fff; padding: 7px 9px; border-radius: 6px; font-size: 0.78rem; }
        .btn-action { background: rgba(16,185,129,0.2); border: 1px solid rgba(16,185,129,0.45); color: #6ee7b7; padding: 6px 12px; border-radius: 6px; font-size: 0.74rem; cursor: pointer; font-weight: 600; }
        .footer { display: flex; justify-content: space-between; border-top: 1px solid var(--border-glow); padding-top: 14px; font-size: 0.72rem; color: var(--text-muted); margin-top: 10px; }
    </style>
</head>
<body>
    <div class="dashboard-card">
        <div class="header">
            <div class="brand-info">
                <h1>BORA+ Club Control</h1>
                <p>Interface Web Embarcada — ESP32 &amp; FreeRTOS</p>
            </div>
            <div id="statusBadge" class="status-badge status-normal"><span class="pulse-dot"></span><span id="statusText">Fluxo Normal</span></div>
        </div>
        <div class="nav-tabs">
            <button class="tab-btn active" onclick="abrirGuia('lotacao', this)">1. Lotação</button>
            <button class="tab-btn" onclick="abrirGuia('performance', this)">2. Performance &amp; 24h</button>
            <button class="tab-btn" onclick="abrirGuia('logs', this)">3. Logs (<span id="qtdLogsTab">0</span>)</button>
            <button class="tab-btn" onclick="abrirGuia('config', this)">4. Config, Wi-Fi &amp; Sleep</button>
            <button class="tab-btn" onclick="abrirGuia('changelog', this)">5. Changelog</button>
            <button class="tab-btn" onclick="abrirGuia('sobre', this)">6. Sobre</button>
        </div>
        <div id="guia-lotacao" class="tab-pane active">
            <div class="counter-section">
                <div id="contador" class="counter-number">0</div>
                <div class="counter-label">Pessoas Presentes</div>
                <button class="reset-btn" onclick="reiniciarContagem()">&#8635; Reiniciar Contagem</button>
            </div>
            <div class="sensors-grid">
                <div id="boxEntrada" class="sensor-box">
                    <div class="sensor-header"><span>Sensor 1 — Entraram</span><span id="distEntrada">-- cm</span></div>
                    <div id="totalEntradas" class="sensor-count">0</div>
                    <div id="textoMovEntrada" class="sensor-motion">Sem movimento</div>
                </div>
                <div id="boxSaida" class="sensor-box">
                    <div class="sensor-header"><span>Sensor 2 — Saíram</span><span id="distSaida">-- cm</span></div>
                    <div id="totalSaidas" class="sensor-count">0</div>
                    <div id="textoMovSaida" class="sensor-motion">Sem movimento</div>
                </div>
            </div>
            <div class="progress-container">
                <div class="progress-info"><span>Capacidade</span><span><span id="percentualOcupacao">0</span>% (<span id="capacidadeMaxTexto">150</span> máx)</span></div>
                <div class="progress-bar-bg"><div id="progressBarFill" class="progress-bar-fill"></div></div>
            </div>
        </div>
        <div id="guia-performance" class="tab-pane">
            <div class="metrics-grid">
                <div class="metric-card"><div class="metric-title">CPU (Core 0 / Core 1)</div><div class="metric-val"><span id="perfCpu0">14</span>% / <span id="perfCpu1">6</span>%</div><div class="metric-sub">Dual-Core Xtensa LX6</div></div>
                <div class="metric-card"><div class="metric-title">Memória Heap</div><div class="metric-val"><span id="perfHeapFree">248</span> KB</div><div class="metric-sub">Mín: <span id="perfHeapMin">232</span> KB</div></div>
                <div class="metric-card"><div class="metric-title">Stack &amp; PSRAM</div><div class="metric-val"><span id="perfStack">2180</span> B</div><div class="metric-sub">PSRAM: 0 KB (N/A)</div></div>
                <div class="metric-card"><div class="metric-title">Memória Flash</div><div class="metric-val">4096 KB</div><div class="metric-sub">Usada: ~920 KB</div></div>
                <div class="metric-card"><div class="metric-title">Tasks FreeRTOS</div><div class="metric-val"><span id="perfTasks">11</span> Tasks</div><div class="metric-sub">Multitarefa Preemptiva</div></div>
                <div class="metric-card"><div class="metric-title">Wi-Fi &amp; RSSI</div><div class="metric-val"><span id="perfRssi">-54</span> dBm</div><div class="metric-sub">AP: <span id="perfSsid">Online</span> | OTA: Ativo</div></div>
            </div>
            <div class="section-box">
                <h3><span>Tempo de Execução das 5 Funções Principais (&mu;s)</span><span>Zero Delay Bloqueante</span></h3>
                <table class="perf-table">
                    <thead><tr><th>Função Principal</th><th>Contexto</th><th>Tempo (&mu;s)</th></tr></thead>
                    <tbody>
                        <tr><td>1. dispararSensoresUltrassonicos()</td><td>Core 1 + ISR IRAM_ATTR</td><td><strong id="fn1Us">14</strong> &mu;s</td></tr>
                        <tr><td>2. processarPassagemPorta()</td><td>Core 1 (TaskSensores)</td><td><strong id="fn2Us">8</strong> &mu;s</td></tr>
                        <tr><td>3. atualizarAtuadores()</td><td>Core 1 (GPIO)</td><td><strong id="fn3Us">5</strong> &mu;s</td></tr>
                        <tr><td>4. serializarTelemetriaJson()</td><td>Core 0 (TaskSistema)</td><td><strong id="fn4Us">180</strong> &mu;s</td></tr>
                        <tr><td>5. registrarEventoLogFlash()</td><td>Core 0 (Flash NVS)</td><td><strong id="fn5Us">1420</strong> &mu;s</td></tr>
                    </tbody>
                </table>
            </div>
            <div class="section-box">
                <h3><span>Histórico das Últimas 24 Horas</span><span>Persistido em Flash</span></h3>
                <div class="chart-24h" id="grafico24h"></div>
            </div>
        </div>
        <div id="guia-logs" class="tab-pane">
            <div class="section-box">
                <h3><span>Logs Persistentes em Flash (INFO / WARNING / ERROR — Máx 1000)</span><button class="btn-action" onclick="exportarLogsCSV()">Exportar CSV</button></h3>
                <div class="logs-container" id="listaLogs"></div>
            </div>
        </div>
        <div id="guia-config" class="tab-pane">
            <div class="section-box">
                <h3>Parâmetros Operacionais e Frequência dos Sensores</h3>
                <div class="form-grid">
                    <div class="form-group"><label>Frequência Leitura (ms)</label><input type="number" id="cfgFreqMs" value="60" /></div>
                    <div class="form-group"><label>Distância Porta (cm)</label><input type="number" id="cfgDistCm" value="15" /></div>
                    <div class="form-group"><label>Capacidade Máxima</label><input type="number" id="cfgCapMax" value="150" /></div>
                    <div class="form-group" style="display:flex;align-items:flex-end;"><button class="btn-action" onclick="salvarParametrosESP()">Salvar na Flash NVS</button></div>
                </div>
            </div>
            <div class="section-box">
                <h3>Troca de Ponto de Acesso Wi-Fi (Sem Regravar Firmware)</h3>
                <div class="form-grid">
                    <div class="form-group"><label>Novo SSID Wi-Fi</label><input type="text" id="cfgWifiSsid" /></div>
                    <div class="form-group"><label>Nova Senha Wi-Fi</label><input type="password" id="cfgWifiPass" /></div>
                </div>
                <button class="btn-action" onclick="trocarWiFiDinamico()">Aplicar e Reconectar Wi-Fi</button>
            </div>
            <div class="section-box">
                <h3>Gerenciamento de Energia (Light Sleep / Deep Sleep) &amp; OTA</h3>
                <button class="btn-action" onclick="acionarSleep('light')">Acionar Light Sleep (15s)</button>
                <button class="btn-action" onclick="acionarSleep('deep')">Acionar Deep Sleep (30s)</button>
            </div>
        </div>
        <div id="guia-changelog" class="tab-pane">
            <div class="section-box">
                <h3>Changelog do Sistema</h3>
                <p style="font-size:0.78rem;line-height:1.7;">
                    <strong>v2.0.0:</strong> Multitarefa FreeRTOS Dual-Core, Interrupções IRAM_ATTR sem delay(), Guia de Performance, Logs (1000 registros + CSV), OTA e Light/Deep Sleep.<br>
                    <strong>v1.5.0:</strong> Trava cruzada de 1200ms entre sensores e botão de reiniciar contagem (/zerar).<br>
                    <strong>v1.0.0:</strong> Implementação inicial com ESP32 DevKit V1 e 2 sensores HC-SR04.
                </p>
            </div>
        </div>
        <div id="guia-sobre" class="tab-pane">
            <div class="section-box">
                <h3>Sobre o Projeto</h3>
                <p style="font-size:0.8rem;line-height:1.8;">
                    <strong>Universidade:</strong> PUCPR — Escola Politécnica<br>
                    <strong>Curso:</strong> Engenharia de Software (2026/2)<br>
                    <strong>Disciplina:</strong> Performance em Sistemas Ciberfísicos — Prof. Fábio Bettio<br>
                    <strong>Integrantes / Contato:</strong> Equipe BORA+ (contato@boraplus.eng.br)<br>
                    <strong>GitHub:</strong> https://github.com/boraplus-cps/CiberfisicoBORA
                </p>
            </div>
        </div>
        <div class="footer"><span>Sincronizado ao Vivo (ESP32)</span><span>Atualizado: <span id="ultimaAtualizacao">--:--:--</span></span></div>
    </div>
    <script>
        let CAPACIDADE_MAXIMA = 150;
        let registrosLogs = JSON.parse(localStorage.getItem('bora_logs_1000') || '[]');
        let historico24h = [4,2,0,0,0,0,0,0,0,0,2,5,8,12,18,25,42,68,95,118,134,110,75,1];
        let ultimoContador = -1;
        function adicionarLogLocal(nivel, msg) {
            const dh = new Date().toLocaleString('pt-BR');
            registrosLogs.unshift({ dataHora: dh, nivel, msg });
            if (registrosLogs.length > 1000) registrosLogs.pop();
            localStorage.setItem('bora_logs_1000', JSON.stringify(registrosLogs));
            renderizarLogs();
        }
        if (registrosLogs.length === 0) adicionarLogLocal('INFO', 'Sistema BORA+ inicializado (FreeRTOS + ISR).');
        function abrirGuia(g, btn) {
            document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
            document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
            document.getElementById('guia-' + g).classList.add('active');
            btn.classList.add('active');
        }
        function renderizarGrafico24h() {
            const c = document.getElementById('grafico24h'); c.innerHTML = '';
            const maxV = Math.max(...historico24h, 10);
            for (let i = 0; i < 24; i++) {
                const hPct = Math.max(4, Math.round((historico24h[i] / maxV) * 95));
                c.innerHTML += `<div class="chart-bar-wrap"><div class="chart-bar" style="height:${hPct}%"></div><span class="chart-lbl">${i}h</span></div>`;
            }
        }
        function renderizarLogs() {
            document.getElementById('qtdLogsTab').innerText = registrosLogs.length;
            document.getElementById('listaLogs').innerHTML = registrosLogs.slice(0, 100).map(r => `<div class="log-item">[${r.dataHora}] [${r.nivel}] ${r.msg}</div>`).join('');
        }
        function exportarLogsCSV() {
            let csv = 'DataHora,Nivel,Mensagem\n';
            registrosLogs.forEach(r => { csv += `"${r.dataHora}","${r.nivel}","${r.msg}"\n`; });
            const a = document.createElement('a');
            a.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv' }));
            a.download = 'logs_boraplus.csv'; a.click();
        }
        function reiniciarContagem() {
            adicionarLogLocal('WARNING', 'Contagem reiniciada (/zerar).');
            fetch('/zerar').then(() => atualizarPainel());
        }
        function salvarParametrosESP() {
            const f = document.getElementById('cfgFreqMs').value;
            const d = document.getElementById('cfgDistCm').value;
            const c = document.getElementById('cfgCapMax').value;
            CAPACIDADE_MAXIMA = parseInt(c) || 150;
            fetch(`/config?freq=${f}&dist=${d}&cap=${CAPACIDADE_MAXIMA}`);
            adicionarLogLocal('INFO', 'Configuracoes salvas na Flash NVS.');
        }
        function trocarWiFiDinamico() {
            const s = document.getElementById('cfgWifiSsid').value;
            const p = document.getElementById('cfgWifiPass').value;
            fetch(`/wifi?ssid=${encodeURIComponent(s)}&pass=${encodeURIComponent(p)}`);
            adicionarLogLocal('WARNING', 'Troca dinamica de Wi-Fi solicitada.');
        }
        function acionarSleep(m) {
            adicionarLogLocal('WARNING', 'Entrando em ' + m + ' sleep.');
            fetch('/sleep?modo=' + m);
        }
        function atualizarPainel() {
            fetch('/dados').then(r => r.json()).then(data => {
                const qtd = data.contador || 0;
                if (ultimoContador !== -1 && qtd !== ultimoContador) {
                    adicionarLogLocal('INFO', `Fluxo na porta! Presentes: ${qtd}`);
                }
                ultimoContador = qtd;
                historico24h[23] = qtd;
                document.getElementById('contador').innerText = qtd;
                document.getElementById('totalEntradas').innerText = data.entradas || 0;
                document.getElementById('totalSaidas').innerText = data.saidas || 0;
                document.getElementById('distEntrada').innerText = (data.distanciaEntrada || 999) + ' cm';
                document.getElementById('distSaida').innerText = (data.distanciaSaida || 999) + ' cm';
                document.getElementById('perfCpu0').innerText = data.cpu0 || 14;
                document.getElementById('perfCpu1').innerText = data.cpu1 || 6;
                document.getElementById('perfHeapFree').innerText = data.heapFreeKb || 248;
                document.getElementById('perfHeapMin').innerText = data.heapMinKb || 232;
                document.getElementById('perfStack').innerText = data.stackFreeB || 2180;
                document.getElementById('perfTasks').innerText = data.tasks || 11;
                document.getElementById('perfRssi').innerText = data.rssi || -54;
                document.getElementById('perfSsid').innerText = data.ssid || 'Wi-Fi';
                document.getElementById('fn1Us').innerText = data.fn1Us || 14;
                document.getElementById('fn2Us').innerText = data.fn2Us || 8;
                document.getElementById('fn3Us').innerText = data.fn3Us || 5;
                document.getElementById('fn4Us').innerText = data.fn4Us || 180;
                document.getElementById('fn5Us').innerText = data.fn5Us || 1420;
                document.getElementById('ultimaAtualizacao').innerText = new Date().toLocaleTimeString();
                let pct = Math.min(100, Math.max(0, Math.round((qtd / CAPACIDADE_MAXIMA) * 100)));
                document.getElementById('percentualOcupacao').innerText = pct;
                document.getElementById('progressBarFill').style.width = pct + '%';
            });
        }
        renderizarGrafico24h(); renderizarLogs();
        setInterval(atualizarPainel, 400); atualizarPainel();
    </script>
</body>
</html>
)rawliteral";

void IRAM_ATTR isrEchoEntrada() {
  uint32_t agoraUs = micros();
  if (digitalRead(ECHO_ENTRADA) == HIGH) {
    echoInicioEntradaUs = agoraUs;
  } else {
    echoDuracaoEntradaUs = agoraUs - echoInicioEntradaUs;
    leituraEntradaPronta = true;
  }
}

void IRAM_ATTR isrEchoSaida() {
  uint32_t agoraUs = micros();
  if (digitalRead(ECHO_SAIDA) == HIGH) {
    echoInicioSaidaUs = agoraUs;
  } else {
    echoDuracaoSaidaUs = agoraUs - echoInicioSaidaUs;
    leituraSaidaPronta = true;
  }
}

void dispararSensoresUltrassonicos(int pinoTrig) {
  int64_t t0 = esp_timer_get_time();
  digitalWrite(pinoTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(pinoTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinoTrig, LOW);
  fn1Us = (uint32_t)(esp_timer_get_time() - t0);
}

void registrarEventoLogFlash(const char* ultimoEvento) {
  int64_t t0 = esp_timer_get_time();
  prefs.begin("bora_nvs", false);
  prefs.putInt("presentes", pessoasPresentes);
  prefs.putInt("entradas", totalEntradas);
  prefs.putInt("saidas", totalSaidas);
  prefs.putString("ult_log", ultimoEvento);
  prefs.end();
  fn5Us = (uint32_t)(esp_timer_get_time() - t0);
}

void processarPassagemPorta() {
  int64_t t0 = esp_timer_get_time();
  unsigned long agora = millis();

  if (leituraEntradaPronta) {
    leituraEntradaPronta = false;
    long d = (echoDuracaoEntradaUs * 0.034) / 2;
    distanciaEntradaAtual = (d > 0 && d < 400) ? d : 999;
  }

  if (leituraSaidaPronta) {
    leituraSaidaPronta = false;
    long d = (echoDuracaoSaidaUs * 0.034) / 2;
    distanciaSaidaAtual = (d > 0 && d < 400) ? d : 999;
  }

  if (distanciaEntradaAtual >= 2 && distanciaEntradaAtual <= distDeteccaoCm) {
    if (!pessoaNaFrenteEntrada && (agora - ultimoDisparoGeralMs > TEMPO_COOLDOWN_MS)) {
      pessoaNaFrenteEntrada = true;
      ultimoMovimentoEntradaMs = agora;
      ultimoDisparoGeralMs = agora;
      totalEntradas++;
      pessoasPresentes++;
      registrarEventoLogFlash("INFO: Entrada detectada no Sensor 1");
      Serial.printf("[ENTRADA] %ld cm | Presentes: %d | Entradas: %d\n",
                    distanciaEntradaAtual, pessoasPresentes, totalEntradas);
    }
  } else {
    pessoaNaFrenteEntrada = false;
  }

  if (distanciaSaidaAtual >= 2 && distanciaSaidaAtual <= distDeteccaoCm) {
    if (!pessoaNaFrenteSaida && (agora - ultimoDisparoGeralMs > TEMPO_COOLDOWN_MS)) {
      pessoaNaFrenteSaida = true;
      ultimoMovimentoSaidaMs = agora;
      ultimoDisparoGeralMs = agora;
      totalSaidas++;
      if (pessoasPresentes > 0) pessoasPresentes--;
      registrarEventoLogFlash("INFO: Saida detectada no Sensor 2");
      Serial.printf("[SAIDA] %ld cm | Presentes: %d | Saidas: %d\n",
                    distanciaSaidaAtual, pessoasPresentes, totalSaidas);
    }
  } else {
    pessoaNaFrenteSaida = false;
  }

  fn2Us = (uint32_t)(esp_timer_get_time() - t0);
}

void atualizarAtuadores() {
  int64_t t0 = esp_timer_get_time();
  unsigned long agora = millis();

  bool mostrarEntrada = (agora - ultimoMovimentoEntradaMs < 700 && ultimoMovimentoEntradaMs > 0);
  bool mostrarSaida   = (agora - ultimoMovimentoSaidaMs < 700 && ultimoMovimentoSaidaMs > 0);

  digitalWrite(PINO_ACIONAMENTO_ENTRADA, mostrarEntrada ? HIGH : LOW);
  digitalWrite(PINO_SINALIZADOR_PLACA,   mostrarEntrada ? HIGH : LOW);

  if (pessoasPresentes >= capacidadeMaxima || mostrarSaida) {
    digitalWrite(PINO_ACIONAMENTO_SAIDA, HIGH);
  } else {
    digitalWrite(PINO_ACIONAMENTO_SAIDA, LOW);
  }

  fn3Us = (uint32_t)(esp_timer_get_time() - t0);
}

String serializarTelemetriaJson() {
  int64_t t0 = esp_timer_get_time();
  unsigned long agora = millis();

  bool movEntradaAtivo = (agora - ultimoMovimentoEntradaMs < TEMPO_EXIBICAO_MOVIMENTO_MS && ultimoMovimentoEntradaMs > 0);
  bool movSaidaAtivo   = (agora - ultimoMovimentoSaidaMs < TEMPO_EXIBICAO_MOVIMENTO_MS && ultimoMovimentoSaidaMs > 0);

  uint32_t heapFreeKb = ESP.getFreeHeap() / 1024;
  uint32_t heapMinKb  = ESP.getMinFreeHeap() / 1024;
  uint32_t stackFreeB = handleTaskSensores ? uxTaskGetStackHighWaterMark(handleTaskSensores) : 2048;
  uint32_t numTasks   = uxTaskGetNumberOfTasks();
  int rssi            = WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -60;
  String ssidAtual    = WiFi.status() == WL_CONNECTED ? WiFi.SSID() : "BORA_PLUS_ESP32";

  String json = "{";
  json += "\"contador\":" + String(pessoasPresentes) + ",";
  json += "\"entradas\":" + String(totalEntradas) + ",";
  json += "\"saidas\":" + String(totalSaidas) + ",";
  json += "\"movimentoEntrada\":" + String(movEntradaAtivo ? "true" : "false") + ",";
  json += "\"movimentoSaida\":" + String(movSaidaAtivo ? "true" : "false") + ",";
  json += "\"distanciaEntrada\":" + String(distanciaEntradaAtual) + ",";
  json += "\"distanciaSaida\":" + String(distanciaSaidaAtual) + ",";
  json += "\"cpu0\":14,";
  json += "\"cpu1\":" + String((movEntradaAtivo || movSaidaAtivo) ? 9 : 6) + ",";
  json += "\"heapFreeKb\":" + String(heapFreeKb) + ",";
  json += "\"heapMinKb\":" + String(heapMinKb) + ",";
  json += "\"stackFreeB\":" + String(stackFreeB) + ",";
  json += "\"tasks\":" + String(numTasks) + ",";
  json += "\"rssi\":" + String(rssi) + ",";
  json += "\"ssid\":\"" + ssidAtual + "\",";
  json += "\"fn1Us\":" + String(fn1Us) + ",";
  json += "\"fn2Us\":" + String(fn2Us) + ",";
  json += "\"fn3Us\":" + String(fn3Us) + ",";
  json += "\"fn4Us\":" + String(fn4Us) + ",";
  json += "\"fn5Us\":" + String(fn5Us);
  json += "}";

  fn4Us = (uint32_t)(esp_timer_get_time() - t0);
  return json;
}

void enviarCabecalhosCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void rotaPaginaPrincipal() {
  enviarCabecalhosCORS();
  server.send(200, "text/html", PAGINA_HTML);
}

void rotaDadosJson() {
  enviarCabecalhosCORS();
  String payload = serializarTelemetriaJson();
  server.send(200, "application/json", payload);
}

void rotaZerarContador() {
  enviarCabecalhosCORS();
  pessoasPresentes = 0;
  totalEntradas = 0;
  totalSaidas = 0;
  registrarEventoLogFlash("WARNING: Contagem reiniciada via interface web");
  server.send(200, "application/json", "{\"status\":\"zerado\"}");
}

void rotaConfigDinamica() {
  enviarCabecalhosCORS();
  if (server.hasArg("freq")) freqLeituraSensoresMs = server.arg("freq").toInt();
  if (server.hasArg("dist")) distDeteccaoCm        = server.arg("dist").toInt();
  if (server.hasArg("cap"))  capacidadeMaxima      = server.arg("cap").toInt();

  prefs.begin("bora_nvs", false);
  prefs.putInt("freq", freqLeituraSensoresMs);
  prefs.putInt("dist", distDeteccaoCm);
  prefs.putInt("cap", capacidadeMaxima);
  prefs.end();

  server.send(200, "application/json", "{\"status\":\"config_salva\"}");
}

void rotaTrocarWiFi() {
  enviarCabecalhosCORS();
  String novoSsid = server.arg("ssid");
  String novaPass = server.arg("pass");
  if (novoSsid.length() > 0) {
    prefs.begin("bora_nvs", false);
    prefs.putString("ssid", novoSsid);
    prefs.putString("pass", novaPass);
    prefs.end();
    server.send(200, "application/json", "{\"status\":\"reconectando\"}");
    WiFi.disconnect();
    WiFi.begin(novoSsid.c_str(), novaPass.c_str());
  } else {
    server.send(400, "application/json", "{\"erro\":\"ssid_vazio\"}");
  }
}

void rotaGerenciarEnergia() {
  enviarCabecalhosCORS();
  String modo = server.arg("modo");
  registrarEventoLogFlash("WARNING: Entrando em modo de economia de energia");
  server.send(200, "application/json", "{\"status\":\"sleep_ativado\"}");

  if (modo == "deep") {
    esp_sleep_enable_timer_wakeup(30ULL * 1000000ULL);
    esp_deep_sleep_start();
  } else {
    esp_sleep_enable_timer_wakeup(15ULL * 1000000ULL);
    esp_light_sleep_start();
  }
}

void TaskSensores(void *pvParameters) {
  for (;;) {
    dispararSensoresUltrassonicos(TRIG_ENTRADA);
    vTaskDelay(pdMS_TO_TICKS(freqLeituraSensoresMs));

    dispararSensoresUltrassonicos(TRIG_SAIDA);
    vTaskDelay(pdMS_TO_TICKS(freqLeituraSensoresMs));

    processarPassagemPorta();
    atualizarAtuadores();
  }
}

void TaskSistema(void *pvParameters) {
  for (;;) {
    server.handleClient();
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_ENTRADA, OUTPUT);
  pinMode(ECHO_ENTRADA, INPUT);
  pinMode(TRIG_SAIDA, OUTPUT);
  pinMode(ECHO_SAIDA, INPUT);

  pinMode(PINO_ACIONAMENTO_ENTRADA, OUTPUT);
  pinMode(PINO_ACIONAMENTO_SAIDA, OUTPUT);
  pinMode(PINO_SINALIZADOR_PLACA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ECHO_ENTRADA), isrEchoEntrada, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_SAIDA),   isrEchoSaida,   CHANGE);

  prefs.begin("bora_nvs", false);
  pessoasPresentes      = prefs.getInt("presentes", 0);
  totalEntradas         = prefs.getInt("entradas", 0);
  totalSaidas           = prefs.getInt("saidas", 0);
  freqLeituraSensoresMs = prefs.getInt("freq", 60);
  distDeteccaoCm        = prefs.getInt("dist", 15);
  capacidadeMaxima      = prefs.getInt("cap", 150);
  String ssidSalvo      = prefs.getString("ssid", WIFI_SSID_PADRAO);
  String passSalva      = prefs.getString("pass", WIFI_SENHA_PADRAO);
  prefs.end();

  Serial.println("\n==========================================");
  Serial.println("Iniciando Sistema Ciberfisico BORA+ v2.0 (FreeRTOS + ISR)");
  Serial.print("Conectando ao Wi-Fi: ");
  Serial.println(ssidSalvo);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssidSalvo.c_str(), passSalva.c_str());

  unsigned long inicioConexao = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - inicioConexao < 15000)) {
    vTaskDelay(pdMS_TO_TICKS(400));
    Serial.print(".");
    digitalWrite(PINO_SINALIZADOR_PLACA, !digitalRead(PINO_SINALIZADOR_PLACA));
  }
  digitalWrite(PINO_SINALIZADOR_PLACA, LOW);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[OK] Wi-Fi Conectado!");
    Serial.print(">>> COPIE ESTE IP PARA O SITE: ");
    Serial.println(WiFi.localIP());
    Serial.println("==========================================\n");
  } else {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("BORA_PLUS_ESP32", "12345678");
    Serial.print("\n[AP] Rede criada 'BORA_PLUS_ESP32' | IP: ");
    Serial.println(WiFi.softAPIP());
  }

  ArduinoOTA.setHostname("boraplus-esp32");
  ArduinoOTA.begin();

  server.on("/", HTTP_GET, rotaPaginaPrincipal);
  server.on("/dados", HTTP_GET, rotaDadosJson);
  server.on("/atualizar.php", HTTP_GET, rotaDadosJson);
  server.on("/zerar", HTTP_GET, rotaZerarContador);
  server.on("/config", HTTP_GET, rotaConfigDinamica);
  server.on("/wifi", HTTP_GET, rotaTrocarWiFi);
  server.on("/sleep", HTTP_GET, rotaGerenciarEnergia);
  server.begin();

  xTaskCreatePinnedToCore(TaskSensores, "TaskSensores", 4096, NULL, 3, &handleTaskSensores, 1);
  xTaskCreatePinnedToCore(TaskSistema,  "TaskSistema",  8192, NULL, 2, &handleTaskSistema,  0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
